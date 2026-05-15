import argparse
import glob
import os
import serial
import struct
import time

DEFAULT_PORT = '/dev/ttyTHS1'
DEFAULT_BAUDRATE = 115200
DEFAULT_MOTOR_IDS = (1, 2)


def _first_serial_by_id() -> str | None:
    items = sorted(glob.glob('/dev/serial/by-id/*'))
    return items[0] if items else None


def _parse_parity(parity: str) -> str:
    p = parity.strip().upper()
    mapping = {
        'N': serial.PARITY_NONE,
        'E': serial.PARITY_EVEN,
        'O': serial.PARITY_ODD,
        'M': serial.PARITY_MARK,
        'S': serial.PARITY_SPACE,
    }
    if p not in mapping:
        raise ValueError('parity phải là một trong: N, E, O, M, S')
    return mapping[p]


def _parse_stopbits(stopbits: str) -> float:
    s = str(stopbits).strip()
    if s in ('1', '1.0'):
        return serial.STOPBITS_ONE
    if s in ('1.5', '1_5'):
        return serial.STOPBITS_ONE_POINT_FIVE
    if s in ('2', '2.0'):
        return serial.STOPBITS_TWO
    raise ValueError('stopbits phải là: 1, 1.5, hoặc 2')


def _parse_bytesize(bytesize: int) -> int:
    mapping = {
        5: serial.FIVEBITS,
        6: serial.SIXBITS,
        7: serial.SEVENBITS,
        8: serial.EIGHTBITS,
    }
    if bytesize not in mapping:
        raise ValueError('bytesize phải là: 5, 6, 7, hoặc 8')
    return mapping[bytesize]


def _list_serial_ports() -> None:
    candidates = []

    for pattern in ('/dev/serial/by-id/*', '/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyTHS*'):
        candidates.extend(sorted(glob.glob(pattern)))

    if not candidates:
        print('Không tìm thấy cổng serial phổ biến nào trong /dev/.')
        return

    print('Các cổng serial tìm thấy (ưu tiên dùng /dev/serial/by-id/* nếu có):')
    for p in candidates:
        resolved = None
        try:
            resolved = os.path.realpath(p)
        except Exception:
            pass
        if resolved and resolved != p:
            print(f'  - {p} -> {resolved}')
        else:
            print(f'  - {p}')


def _open_serial_from_args(args) -> serial.Serial:
    port = str(args.port)
    if port.lower() == 'auto':
        by_id = _first_serial_by_id()
        if by_id is not None:
            port = by_id
        else:
            port = DEFAULT_PORT

    ser = serial.Serial(
        port=port,
        baudrate=int(args.baudrate),
        timeout=float(args.timeout),
        bytesize=_parse_bytesize(int(args.bytesize)),
        parity=_parse_parity(str(args.parity)),
        stopbits=_parse_stopbits(str(args.stopbits)),
        xonxoff=bool(args.xonxoff),
        rtscts=bool(args.rtscts),
        dsrdtr=bool(args.dsrdtr),
    )

    # Optional: enable RS485 mode controlled by RTS
    if bool(args.rs485):
        try:
            from serial.rs485 import RS485Settings

            ser.rs485_mode = RS485Settings(
                rts_level_for_tx=bool(args.rs485_rts_tx),
                rts_level_for_rx=bool(args.rs485_rts_rx),
                delay_before_tx=float(args.rs485_delay_before_tx),
                delay_before_rx=float(args.rs485_delay_before_rx),
            )
        except Exception as e:
            ser.close()
            raise RuntimeError(
                'Không bật được chế độ RS485 (pyserial/driver không hỗ trợ rs485_mode). '
                'Nếu bạn đang nối TTL UART thẳng vào motor (không qua transceiver RS485), hãy bỏ --rs485.'
            ) from e

    return ser


def _compute_checksum(header: int, addr: int, length: int, type_hi: int, type_lo: int, data_bytes: list[int]) -> int:
    return (header + addr + length + type_hi + type_lo + sum(data_bytes)) & 0xFF


def _read_frame(ser: serial.Serial, *, timeout_s: float) -> tuple[int, int, bytes] | None:
    """Read one protocol frame.

    Returns (addr, cmd_type, data_bytes) on success, or None on timeout.
    """

    header = 0xFE
    footer = 0xFD

    deadline = time.monotonic() + max(0.0, float(timeout_s))

    while time.monotonic() <= deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != header:
            continue

        fixed = ser.read(1 + 1 + 2)  # addr, length, type_hi, type_lo
        if len(fixed) != 4:
            continue
        addr = fixed[0]
        length = fixed[1]
        type_hi = fixed[2]
        type_lo = fixed[3]
        cmd_type = (type_hi << 8) | type_lo

        data = ser.read(length)
        if len(data) != length:
            continue

        tail = ser.read(2)  # checksum, footer
        if len(tail) != 2:
            continue
        checksum = tail[0]
        f = tail[1]
        if f != footer:
            continue

        expected = _compute_checksum(header, addr, length, type_hi, type_lo, list(data))
        if checksum != expected:
            continue

        return addr, cmd_type, data

    return None


def _probe_status(ser: serial.Serial, *, motor_id: int, timeout_s: float, retries: int) -> bool:
    """Send Status (103) and wait for GotStatus (102)."""
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    for _ in range(max(1, int(retries))):
        send_command(ser, motor_id, 103)

        frame = _read_frame(ser, timeout_s=timeout_s)
        if frame is None:
            continue

        addr, cmd_type, data = frame
        if addr != motor_id:
            continue
        if cmd_type != 102:
            continue

        try:
            text = data.decode('utf-8', errors='replace')
        except Exception:
            text = repr(data)

        print('OK: Motor có phản hồi GotStatus (102).')
        print(f'  addr={addr}, text={text}')
        return True

    print('FAIL: Không nhận được phản hồi GotStatus (102).')
    print('  Gợi ý: kiểm tra A/B không đảo, GND chung, đúng baudrate (115200), đúng ID, và module RS485 auto-direction.')
    return False


def _build_packet(addr: int, cmd_type: int, data: int | None = None) -> bytes:
    header = 0xFE
    footer = 0xFD
    type_hi = (cmd_type >> 8) & 0xFF
    type_lo = cmd_type & 0xFF

    data_bytes: list[int] = []
    if data is not None:
        data_bytes = list(struct.pack('>i', int(data)))

    length = len(data_bytes)
    checksum = (header + addr + length + type_hi + type_lo + sum(data_bytes)) & 0xFF
    packet = [header, addr, length, type_hi, type_lo] + data_bytes + [checksum, footer]
    return bytes(packet)


def send_command(ser: serial.Serial, addr: int, cmd_type: int, data: int | None = None, *, sleep_s: float = 0.01) -> None:
    ser.write(_build_packet(addr, cmd_type, data))
    if sleep_s > 0:
        time.sleep(float(sleep_s))


def send_command_multi(
    ser: serial.Serial,
    motor_ids: list[int],
    cmd_type: int,
    data: int | None = None,
    *,
    inter_frame_delay_s: float = 0.0,
    post_write_sleep_s: float = 0.01,
) -> None:
    """Send the same command to multiple motors as tightly as possible.

    Default behavior is to write one frame per ID (similar pacing to toc_do.py).
    Some RS485 transceivers / motor firmwares can be sensitive to back-to-back
    frames without a tiny idle gap; use inter_frame_delay_s to tune.
    """

    ids = [int(x) for x in motor_ids]
    if not ids:
        return

    delay = max(0.0, float(inter_frame_delay_s))
    for index, mid in enumerate(ids):
        ser.write(_build_packet(mid, cmd_type, data))
        # Delay only between frames (not after the last one)
        if delay > 0.0 and index != (len(ids) - 1):
            time.sleep(delay)

    if float(post_write_sleep_s) > 0.0:
        time.sleep(float(post_write_sleep_s))


def _parse_ids(ids_str: str) -> list[int]:
    raw = [x.strip() for x in str(ids_str).split(',') if x.strip()]
    if not raw:
        raise ValueError('ids không được rỗng (vd: --ids 1,2)')

    ids: list[int] = []
    for x in raw:
        try:
            ids.append(int(x))
        except ValueError as e:
            raise ValueError(f'ID không hợp lệ: {x!r}') from e

    uniq = []
    for mid in ids:
        if mid not in uniq:
            uniq.append(mid)

    return uniq


def _parse_id_set(ids_str: str) -> set[int]:
    raw = [x.strip() for x in str(ids_str).split(',') if x.strip()]
    if not raw:
        return set()

    ids: set[int] = set()
    for x in raw:
        try:
            ids.add(int(x))
        except ValueError as e:
            raise ValueError(f'ID không hợp lệ: {x!r}') from e

    return ids


def send_speed_multi(
    ser: serial.Serial,
    motor_ids: list[int],
    base_speed: int,
    *,
    invert_ids: set[int],
    inter_frame_delay_s: float,
    post_write_sleep_s: float = 0.01,
) -> None:
    ids = [int(x) for x in motor_ids]
    delay = max(0.0, float(inter_frame_delay_s))

    for index, mid in enumerate(ids):
        speed = -int(base_speed) if int(mid) in invert_ids else int(base_speed)
        ser.write(_build_packet(mid, 122, speed))
        if delay > 0.0 and index != (len(ids) - 1):
            time.sleep(delay)

    if float(post_write_sleep_s) > 0.0:
        time.sleep(float(post_write_sleep_s))


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Gửi lệnh tốc độ tới 2 motor cùng lúc theo giao thức RS485 (frame FE..FD).'
    )
    parser.add_argument(
        '--port',
        default=DEFAULT_PORT,
        help='Cổng serial. Dùng /dev/ttyTHS1 (Jetson UART), /dev/ttyUSB0 (USB), hoặc /dev/serial/by-id/.... '
        'Có thể dùng "auto" để tự ưu tiên /dev/serial/by-id/*.',
    )
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE, help='Baudrate (mặc định 115200)')
    parser.add_argument(
        '--ids',
        default=','.join(str(x) for x in DEFAULT_MOTOR_IDS),
        help='Danh sách ID, phân tách bởi dấu phẩy (mặc định 1,2). Ví dụ: --ids 1,2',
    )
    parser.add_argument(
        '--invert-ids',
        default=str(DEFAULT_MOTOR_IDS[0]),
        help='Các ID cần đảo dấu vận tốc, phân tách bởi dấu phẩy (mặc định đảo ID=1). Ví dụ: --invert-ids 1',
    )
    parser.add_argument('--list-ports', action='store_true', help='Liệt kê các cổng serial khả dụng rồi thoát')

    # Serial line settings (UART)
    parser.add_argument('--timeout', type=float, default=0.1, help='Timeout đọc (giây)')
    parser.add_argument('--parity', default='N', help='Parity: N/E/O/M/S (mặc định N)')
    parser.add_argument('--stopbits', default='1', help='Stop bits: 1 / 1.5 / 2 (mặc định 1)')
    parser.add_argument('--bytesize', type=int, default=8, help='Data bits: 5/6/7/8 (mặc định 8)')
    parser.add_argument('--rtscts', action='store_true', help='Bật hardware flow control RTS/CTS')
    parser.add_argument('--dsrdtr', action='store_true', help='Bật DSR/DTR (hiếm khi cần)')
    parser.add_argument('--xonxoff', action='store_true', help='Bật software flow control XON/XOFF')

    # RS485 mode
    parser.add_argument('--rs485', action='store_true', help='Bật RS485 mode (RTS điều khiển hướng TX/RX)')
    parser.add_argument('--rs485-rts-tx', action='store_true', help='Mức RTS khi TX (mặc định False)')
    parser.add_argument('--rs485-rts-rx', action='store_true', help='Mức RTS khi RX (mặc định False)')
    parser.add_argument('--rs485-delay-before-tx', type=float, default=0.0, help='Delay trước TX (giây)')
    parser.add_argument('--rs485-delay-before-rx', type=float, default=0.0, help='Delay trước RX (giây)')

    # Sending timing
    parser.add_argument(
        '--inter-frame-delay',
        type=float,
        default=0.01,
        help='Delay giữa 2 frame khi gửi nhiều ID (giây). Mặc định 0.01 để giống nhịp gửi của toc_do.py.',
    )

    # Quick connectivity check
    parser.add_argument('--check-only', action='store_true', help='Chỉ kiểm tra mở được cổng UART/USB rồi thoát')

    # Probe motors
    parser.add_argument('--probe', action='store_true', help='Probe cả 2 motor bằng Status(103) -> GotStatus(102)')
    parser.add_argument('--probe-timeout', type=float, default=0.5, help='Timeout đợi phản hồi probe (giây)')
    parser.add_argument('--probe-retries', type=int, default=3, help='Số lần thử probe')

    args = parser.parse_args()

    if args.list_ports:
        _list_serial_ports()
        return

    motor_ids = _parse_ids(args.ids)
    invert_ids = _parse_id_set(args.invert_ids)

    resolved_port = str(args.port)
    if resolved_port.lower() == 'auto':
        resolved_port = _first_serial_by_id() or DEFAULT_PORT

    print(
        f"Mở cổng nối tiếp {resolved_port} @ {int(args.baudrate)} "
        f"(8N1 mặc định; bytesize={args.bytesize}, parity={args.parity}, stopbits={args.stopbits})..."
    )

    try:
        ser = _open_serial_from_args(args)
    except Exception as e:
        print(f"Lỗi: {e}")
        return

    if args.check_only:
        print('OK: Mở được cổng serial. Đóng lại và thoát (--check-only).')
        try:
            ser.close()
        except Exception:
            pass
        return

    if args.probe:
        all_ok = True
        for mid in motor_ids:
            print(f'--- Probe motor ID={mid} ---')
            ok = _probe_status(
                ser,
                motor_id=int(mid),
                timeout_s=float(args.probe_timeout),
                retries=int(args.probe_retries),
            )
            all_ok = all_ok and ok

        try:
            ser.close()
        except Exception:
            pass

        raise SystemExit(0 if all_ok else 1)

    time.sleep(1)

    print('--- KHỞI TẠO CHẾ ĐỘ TỐC ĐỘ (2 motor) ---')
    send_command_multi(
        ser,
        motor_ids,
        105,
        inter_frame_delay_s=float(args.inter_frame_delay),
    )  # Enable Motor
    send_command_multi(
        ser,
        motor_ids,
        120,
        inter_frame_delay_s=float(args.inter_frame_delay),
    )  # Speed mode

    if invert_ids:
        inv_sorted = ','.join(str(x) for x in sorted(invert_ids))
        print(f'Ghi chú: Các motor sẽ bị đảo dấu vận tốc: {inv_sorted}')
    print('Khởi tạo xong! 2 bánh xe đã sẵn sàng lăn.')
    print('-' * 40)
    print('HƯỚNG DẪN:')
    print('- Nhập số dương (vd: 50000) để QUAY TỚI (cả 2 motor).')
    print('- Nhập số âm (vd: -50000) để QUAY LÙI (cả 2 motor).')
    print('- Nhập số 0 để PHANH DỪNG LẠI.')
    print("- Nhập phím 'q' để tắt động cơ và thoát.")
    print('-' * 40)

    try:
        while True:
            user_input = input('\nNhập vận tốc (counts/s) cho CẢ 2 motor: ')

            if user_input.lower() == 'q':
                print('Đang dừng xe và tắt động cơ...')
                send_speed_multi(
                    ser,
                    motor_ids,
                    0,
                    invert_ids=invert_ids,
                    inter_frame_delay_s=float(args.inter_frame_delay),
                )
                time.sleep(0.5)
                break

            try:
                speed = int(user_input)
                send_speed_multi(
                    ser,
                    motor_ids,
                    speed,
                    invert_ids=invert_ids,
                    inter_frame_delay_s=float(args.inter_frame_delay),
                )

                if speed == 0:
                    print('=> Đã phanh khẩn cấp! (Vận tốc = 0)')
                else:
                    preview = []
                    for mid in motor_ids:
                        s = -speed if int(mid) in invert_ids else speed
                        preview.append(f'ID{mid}:{s}')
                    print('=> Đã gửi: ' + '  '.join(preview))

            except ValueError:
                print('Lỗi: Bạn phải nhập SỐ NGUYÊN để chỉnh ga!')

    finally:
        send_command_multi(
            ser,
            motor_ids,
            121,
            inter_frame_delay_s=float(args.inter_frame_delay),
        )  # StopSpd
        send_command_multi(
            ser,
            motor_ids,
            106,
            inter_frame_delay_s=float(args.inter_frame_delay),
        )  # Disable motor
        try:
            ser.close()
        except Exception:
            pass

    print('Đã thoát chương trình an toàn.')


if __name__ == '__main__':
    main()
