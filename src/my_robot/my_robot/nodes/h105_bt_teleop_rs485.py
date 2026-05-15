#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import os
import struct
import time

import serial

from evdev import InputDevice, ecodes, list_devices
from select import select


def _event_index(path: str) -> int:
    """Extract numeric index from /dev/input/eventX; unknown -> large."""
    base = os.path.basename(str(path))
    if base.startswith('event') and base[5:].isdigit():
        return int(base[5:])
    return 10**9


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


def _open_serial_from_args(args) -> serial.Serial:
    port = str(args.port)
    if port.lower() == 'auto':
        port = _first_serial_by_id() or DEFAULT_PORT

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
    ser.write(_build_packet(int(addr), int(cmd_type), data))
    if float(sleep_s) > 0:
        time.sleep(float(sleep_s))


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

    uniq: list[int] = []
    for mid in ids:
        if mid not in uniq:
            uniq.append(mid)

    return uniq


def _parse_id_set(ids_str: str) -> set[int]:
    raw = [x.strip() for x in str(ids_str).split(',') if x.strip()]
    if not raw:
        return set()

    out: set[int] = set()
    for x in raw:
        try:
            out.add(int(x))
        except ValueError as e:
            raise ValueError(f'ID không hợp lệ: {x!r}') from e

    return out


def _parse_axis(axis: str) -> int:
    if ecodes is None:
        raise RuntimeError('Thiếu evdev/ecodes')

    s = str(axis).strip()
    if s.isdigit() or (s.startswith('-') and s[1:].isdigit()):
        return int(s)

    key = s.upper()
    if not key.startswith('ABS_'):
        key = 'ABS_' + key

    if not hasattr(ecodes, key):
        raise ValueError(f'Axis không hợp lệ: {axis!r}. Ví dụ: ABS_X, ABS_Y, ABS_RX, ABS_RY')

    return int(getattr(ecodes, key))


def _parse_key(key: str) -> int:
    if ecodes is None:
        raise RuntimeError('Thiếu evdev/ecodes')

    s = str(key).strip()
    if s.isdigit() or (s.startswith('-') and s[1:].isdigit()):
        return int(s)

    k = s.upper()
    if not (k.startswith('KEY_') or k.startswith('BTN_')):
        # Common shorthand: A/B from gamepad mapping often surfaces as BTN_0/BTN_1
        k = 'BTN_' + k

    if not hasattr(ecodes, k):
        raise ValueError(f'Key không hợp lệ: {key!r}. Ví dụ: BTN_0, BTN_1, BTN_SOUTH, KEY_A')
    return int(getattr(ecodes, k))


def _find_h105_device(preferred_path: str | None, name_contains: str) -> InputDevice:
    if InputDevice is None or list_devices is None:
        raise RuntimeError(
            'Chưa có thư viện evdev. Cài bằng: sudo apt-get install -y python3-evdev (hoặc pip install evdev).'
        )

    if preferred_path:
        dev = InputDevice(preferred_path)
        return dev

    candidates: list[InputDevice] = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except Exception:
            continue
        if name_contains.lower() in (dev.name or '').lower():
            candidates.append(dev)

    if not candidates:
        names = []
        for path in list_devices():
            try:
                dev = InputDevice(path)
                names.append(f'{dev.path}: {dev.name}')
            except Exception:
                pass
        raise RuntimeError(
            f'Không tìm thấy thiết bị có tên chứa {name_contains!r}. '
            'Gợi ý: kiểm tra đã connect Bluetooth chưa, và bạn có quyền đọc /dev/input/event* không.\n'
            'Thiết bị nhìn thấy:\n  - ' + '\n  - '.join(names)
        )

    # Nếu có nhiều device (keyboard/mouse/consumer control), ưu tiên device có EV_ABS,
    # và luôn chọn event có index nhỏ hơn để ổn định (vd event13 thay vì event15).
    def has_abs(d: InputDevice) -> bool:
        try:
            caps = d.capabilities() or {}
        except Exception:
            caps = {}
        return ecodes.EV_ABS in caps

    abs_candidates = [d for d in candidates if has_abs(d)]
    pool = abs_candidates if abs_candidates else candidates
    pool_sorted = sorted(pool, key=lambda d: _event_index(d.path))
    return pool_sorted[0]


def _clamp(x: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, x))


def _step_towards(current: int, target: int, *, step_up: int, step_down: int) -> int:
    if current == target:
        return current

    if target > current:
        step = max(0, int(step_up))
        if step <= 0:
            return target
        return min(target, current + step)

    step = max(0, int(step_down))
    if step <= 0:
        return target
    return max(target, current - step)


def _normalize_abs_neutral(
    dev: InputDevice,
    code: int,
    value: int,
    *,
    neutral: int,
    deadzone: float,
    invert: bool,
) -> float:
    """Normalize an ABS axis around a *measured neutral* value.

    This is useful when a controller reports a non-centered range (e.g. 0..127)
    or when auto-detect misclassifies an axis as throttle. Neutral is the raw
    value read at startup.

    Maps [neutral - span .. neutral + span] -> [-1..1] where span is the
    maximum distance to either end (min/max) from neutral.
    """
    info = None
    try:
        info = dev.absinfo(code)
    except Exception:
        info = None

    if info is None:
        vmin, vmax = -32768, 32767
    else:
        vmin, vmax = int(info.min), int(info.max)

    span = max(1, max(abs(int(vmax) - int(neutral)), abs(int(neutral) - int(vmin))))
    norm = (float(int(value)) - float(int(neutral))) / float(span)

    if invert:
        norm = -norm

    if abs(norm) < float(deadzone):
        return 0.0

    return _clamp(norm, -1.0, 1.0)


def _send_left_right(
    ser: serial.Serial,
    *,
    left_id: int,
    right_id: int,
    left_speed: int,
    right_speed: int,
    invert_ids: set[int],
    inter_frame_delay_s: float,
) -> None:
    delay = max(0.0, float(inter_frame_delay_s))

    l = -int(left_speed) if int(left_id) in invert_ids else int(left_speed)
    r = -int(right_speed) if int(right_id) in invert_ids else int(right_speed)

    ser.write(_build_packet(int(left_id), 122, l))
    if delay > 0:
        time.sleep(delay)
    ser.write(_build_packet(int(right_id), 122, r))


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Điều khiển 2 bánh xe bằng joystick Bluetooth H105 BT qua /dev/input/event* (evdev) và gửi tốc độ motor RS485.'
    )

    # Joystick
    parser.add_argument('--device', default='', help='Đường dẫn input device (vd: /dev/input/event13). Bỏ trống để auto theo tên.')
    parser.add_argument('--name-contains', default='H105 BT', help='Chuỗi tên để auto tìm thiết bị (mặc định "H105 BT").')
    parser.add_argument('--grab', action='store_true', help='Grab thiết bị để tránh app khác đọc cùng lúc (khuyến nghị).')
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Chỉ đọc input và in tốc độ (không mở serial, không gửi lệnh motor).',
    )

    parser.add_argument('--axis-linear', default='ABS_Y', help='Axis tiến/lùi (mặc định ABS_Y).')
    parser.add_argument('--axis-angular', default='ABS_X', help='Axis trái/phải (mặc định ABS_X).')
    parser.add_argument(
        '--no-invert-linear',
        action='store_true',
        help='Tắt đảo chiều trục tiến/lùi (mặc định: có đảo để gạt lên = tiến).',
    )
    parser.add_argument('--invert-angular', action='store_true', help='Đảo chiều trục trái/phải.')

    parser.add_argument('--deadzone', type=float, default=0.08, help='Deadzone cho joystick (0..1).')

    parser.add_argument('--speed-start', type=int, default=500, help='Tốc độ khởi đầu (counts).')
    parser.add_argument('--speed-step', type=int, default=500, help='Mỗi lần bấm A/B sẽ +/- bấy nhiêu (counts).')
    parser.add_argument('--speed-limit', type=int, default=50000, help='Giới hạn tốc độ tối đa khi dùng --fixed-speed-buttons (counts).')
    parser.add_argument('--speed-inc-key', default='BTN_SOUTH', help='Phím tăng tốc (mặc định BTN_SOUTH = A).')
    parser.add_argument('--speed-dec-key', default='BTN_EAST', help='Phím giảm tốc (mặc định BTN_EAST = B).')
    parser.add_argument(
        '--input-timeout',
        type=float,
        default=0.0,
        help='Fail-safe: nếu input ở trạng thái NEUTRAL và quá thời gian này không có event -> stop (giây). 0 = tắt.',
    )
    parser.add_argument('--heartbeat', type=float, default=0.2, help='Gửi lại tốc độ định kỳ dù không đổi (giây).')

    parser.add_argument('--turn-scale', type=float, default=0.8, help='Hệ số cho trái/phải (0..1..).')

    parser.add_argument(
        '--debug-input',
        action='store_true',
        help='In thông tin ABS min/max và raw axis để debug mapping.',
    )

    parser.add_argument(
        '--ramp-up-step',
        type=int,
        default=500,
        help='Bước tăng tốc mỗi vòng lặp (counts). 0 = tắt ramp (nhảy thẳng). Mặc định 500.',
    )
    parser.add_argument(
        '--ramp-down-step',
        type=int,
        default=2000,
        help='Bước giảm tốc mỗi vòng lặp (counts). 0 = giảm ngay lập tức. Mặc định 2000.',
    )

    # Motor/Serial
    parser.add_argument('--port', default=DEFAULT_PORT, help='Cổng serial (vd: /dev/ttyTHS1, /dev/ttyUSB0, hoặc auto).')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE, help='Baudrate (mặc định 115200).')
    parser.add_argument('--timeout', type=float, default=0.1, help='Serial read timeout (giây).')
    parser.add_argument('--parity', default='N', help='Parity: N/E/O/M/S (mặc định N).')
    parser.add_argument('--stopbits', default='1', help='Stop bits: 1 / 1.5 / 2 (mặc định 1).')
    parser.add_argument('--bytesize', type=int, default=8, help='Data bits: 5/6/7/8 (mặc định 8).')
    parser.add_argument('--rtscts', action='store_true')
    parser.add_argument('--dsrdtr', action='store_true')
    parser.add_argument('--xonxoff', action='store_true')

    parser.add_argument('--rs485', action='store_true', help='Bật RS485 mode (RTS điều khiển hướng TX/RX).')
    parser.add_argument('--rs485-rts-tx', action='store_true')
    parser.add_argument('--rs485-rts-rx', action='store_true')
    parser.add_argument('--rs485-delay-before-tx', type=float, default=0.0)
    parser.add_argument('--rs485-delay-before-rx', type=float, default=0.0)

    parser.add_argument('--ids', default=','.join(str(x) for x in DEFAULT_MOTOR_IDS), help='Danh sách motor ID, ví dụ: 1,2')
    parser.add_argument('--invert-ids', default=str(DEFAULT_MOTOR_IDS[0]), help='ID cần đảo dấu vận tốc, ví dụ: 1')
    parser.add_argument('--inter-frame-delay', type=float, default=0.01, help='Delay giữa 2 frame (giây).')

    args = parser.parse_args()

    preferred_device = str(args.device).strip() or None
    dev = _find_h105_device(preferred_device, str(args.name_contains))

    print(f'Đang dùng input device: {dev.path} | {dev.name}')
    if args.grab:
        try:
            dev.grab()
            print('Đã grab thiết bị input.')
        except Exception as e:
            print(f'Không grab được device (bỏ qua): {e}')

    caps = dev.capabilities() or {}
    if ecodes.EV_ABS not in caps:
        raise SystemExit('Thiết bị input không có EV_ABS (analog). Hãy chọn đúng /dev/input/event* của joystick.')

    print('Mode: FIXED SPEED (A/B)')

    linear_code = _parse_axis(args.axis_linear)
    angular_code = _parse_axis(args.axis_angular)

    inc_key_code = _parse_key(args.speed_inc_key)
    dec_key_code = _parse_key(args.speed_dec_key)

    # Read initial ABS values to use as NEUTRAL baseline.
    raw_linear_init = 0
    raw_angular_init = 0
    try:
        raw_linear_init = int(dev.absinfo(linear_code).value)
    except Exception:
        raw_linear_init = 0
    try:
        raw_angular_init = int(dev.absinfo(angular_code).value)
    except Exception:
        raw_angular_init = 0

    if bool(args.debug_input):
        try:
            lin_info = dev.absinfo(linear_code)
            ang_info = dev.absinfo(angular_code)
            print(f'ABS linear code={linear_code} info={lin_info}')
            print(f'ABS angular code={angular_code} info={ang_info}')
        except Exception as e:
            print(f'Không đọc được absinfo: {e}')

    # Default ergonomics: ABS_Y thường là - khi gạt lên, nên invert để tiến là dương.
    invert_linear = not bool(args.no_invert_linear)
    invert_angular = bool(args.invert_angular)

    motor_ids = _parse_ids(args.ids)
    if len(motor_ids) != 2:
        raise SystemExit('Cần đúng 2 motor ID. Ví dụ: --ids 1,2')

    left_id, right_id = motor_ids[0], motor_ids[1]
    invert_ids = _parse_id_set(args.invert_ids)

    ser = None if bool(args.dry_run) else _open_serial_from_args(args)

    try:
        time.sleep(0.2)
        if ser is not None:
            # Init motors: Enable + speed mode
            send_command(ser, left_id, 105, sleep_s=0.01)
            send_command(ser, right_id, 105, sleep_s=float(args.inter_frame_delay))
            send_command(ser, left_id, 120, sleep_s=0.01)
            send_command(ser, right_id, 120, sleep_s=float(args.inter_frame_delay))
            print('Đã init motor. Bắt đầu đọc joystick... (Ctrl+C để thoát)')
        else:
            print('DRY-RUN: Không mở serial/motor. Chỉ đọc input và in tốc độ.')

        raw_linear = int(raw_linear_init)
        raw_angular = int(raw_angular_init)
        got_linear = True
        got_angular = True

        last_event_t = time.monotonic()
        last_send_t = 0.0
        last_sent = (None, None)

        ramp_left = 0
        ramp_right = 0

        timeout_s = max(0.0, float(args.input_timeout))
        heartbeat_s = max(0.02, float(args.heartbeat))
        poll_s = 0.02

        fixed_speed = max(0, int(args.speed_start))
        speed_step = max(0, int(args.speed_step))
        speed_limit = max(0, int(args.speed_limit))
        print(
            'Fixed-speed mode ON (default): '
            f'speed={fixed_speed}, step={speed_step}, limit={speed_limit}, '
            f'inc={args.speed_inc_key}, dec={args.speed_dec_key}'
        )

        while True:
            r, _, _ = select([dev], [], [], poll_s)
            now = time.monotonic()

            if r:
                try:
                    for event in dev.read():
                        # Speed adjust buttons
                        if event.type == ecodes.EV_KEY and event.value == 1:
                            if bool(args.debug_input):
                                key_name = None
                                try:
                                    key_name = ecodes.KEY.get(int(event.code), None)
                                except Exception:
                                    key_name = None
                                if key_name:
                                    print(f'[key] code={int(event.code)} name={key_name}')
                                else:
                                    print(f'[key] code={int(event.code)}')
                            if int(event.code) == int(inc_key_code):
                                fixed_speed = min(speed_limit, fixed_speed + speed_step)
                                print(f'[speed] {fixed_speed}')
                            elif int(event.code) == int(dec_key_code):
                                fixed_speed = max(0, fixed_speed - speed_step)
                                print(f'[speed] {fixed_speed}')

                        if event.type == ecodes.EV_ABS:
                            if event.code == linear_code:
                                raw_linear = int(event.value)
                                last_event_t = now
                                got_linear = True
                            elif event.code == angular_code:
                                raw_angular = int(event.value)
                                last_event_t = now
                                got_angular = True
                except OSError as e:
                    print(f'Input device bị mất kết nối: {e}')
                    break

            if not got_linear:
                lin = 0.0
            else:
                lin = _normalize_abs_neutral(
                    dev,
                    linear_code,
                    raw_linear,
                    neutral=int(raw_linear_init),
                    deadzone=float(args.deadzone),
                    invert=invert_linear,
                )

            if not got_angular:
                ang = 0.0
            else:
                ang = _normalize_abs_neutral(
                    dev,
                    angular_code,
                    raw_angular,
                    neutral=int(raw_angular_init),
                    deadzone=float(args.deadzone),
                    invert=invert_angular,
                )

            if abs(lin) > 0.0 or abs(ang) > 0.0:
                last_event_t = now

            if timeout_s > 0.0 and (now - last_event_t) > timeout_s:
                lin = 0.0
                ang = 0.0

            ang *= float(args.turn_scale)
            lin = _clamp(lin, -1.0, 1.0)
            ang = _clamp(ang, -1.0, 1.0)

            # Direction from lin sign; turning from ang magnitude
            lin_dir = 0.0
            if lin > 0.05:
                lin_dir = 1.0
            elif lin < -0.05:
                lin_dir = -1.0

            left = _clamp(lin_dir + ang, -1.0, 1.0)
            right = _clamp(lin_dir - ang, -1.0, 1.0)
            left_target = int(round(left * int(fixed_speed)))
            right_target = int(round(right * int(fixed_speed)))

            # Ramp: tăng/giảm từ từ để tránh “nhảy” tốc độ.
            ramp_left = _step_towards(
                ramp_left,
                left_target,
                step_up=int(args.ramp_up_step),
                step_down=int(args.ramp_down_step),
            )
            ramp_right = _step_towards(
                ramp_right,
                right_target,
                step_up=int(args.ramp_up_step),
                step_down=int(args.ramp_down_step),
            )

            should_send = False
            changed = False
            if last_sent != (ramp_left, ramp_right):
                should_send = True
                changed = True
            elif (now - float(last_send_t)) >= heartbeat_s:
                should_send = True

            if should_send:
                if ser is not None:
                    _send_left_right(
                        ser,
                        left_id=left_id,
                        right_id=right_id,
                        left_speed=ramp_left,
                        right_speed=ramp_right,
                        invert_ids=invert_ids,
                        inter_frame_delay_s=float(args.inter_frame_delay),
                    )
                    # Print only when command changes to avoid spamming.
                    if changed:
                        print(
                            f'L={ramp_left:7d}  R={ramp_right:7d}  '
                            f'(speed={fixed_speed}, lin={lin:+.2f} ang={ang:+.2f})'
                        )
                else:
                    print(
                        f'L={ramp_left:7d}  R={ramp_right:7d}  '
                        f'target(L={left_target:7d} R={right_target:7d})  '
                        + f'(speed={fixed_speed}, lin={lin:+.2f} ang={ang:+.2f} rawL={raw_linear} rawA={raw_angular})'
                    )
                last_sent = (ramp_left, ramp_right)
                last_send_t = now

    except KeyboardInterrupt:
        print('\nĐang dừng...')
    finally:
        try:
            # Stop speed + disable
            if ser is not None:
                send_command(ser, left_id, 122, 0, sleep_s=0.0)
                send_command(ser, right_id, 122, 0, sleep_s=float(args.inter_frame_delay))
                send_command(ser, left_id, 121, sleep_s=0.01)
                send_command(ser, right_id, 121, sleep_s=float(args.inter_frame_delay))
                send_command(ser, left_id, 106, sleep_s=0.01)
                send_command(ser, right_id, 106, sleep_s=float(args.inter_frame_delay))
        except Exception:
            pass
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        try:
            if args.grab:
                dev.ungrab()
        except Exception:
            pass


if __name__ == '__main__':
    main()
