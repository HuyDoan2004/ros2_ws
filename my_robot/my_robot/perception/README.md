# my_robot.perception – Hiệu chỉnh Camera–LiDAR (Checkerboard)

Tài liệu này mô tả **cách hiệu chỉnh ngoại (extrinsic)** giữa **camera depth D435i** và **LiDAR 2D (C1M1)** bằng **checkerboard**, sử dụng các node trong thư mục `perception` của package `my_robot` trên **ROS 2 Humble**.

## 1. Các file liên quan

### Thuật toán & xử lý dữ liệu

- `cam_lidar_calib_example.py`
  - Demo thuật toán Kabsch/Umeyama với dữ liệu giả lập (synthetic) để hiểu trực giác.
  - Không phụ thuộc ROS, chỉ dùng `numpy`.
- `cam_lidar_calib_from_data.py`
  - Script đọc dữ liệu cặp điểm đã ghi (`points_L`, `points_C`) và tính transform **Camera ← LiDAR**:
    - Ma trận quay `R` (3x3).
    - Vector tịnh tiến `t = [t_x, t_y, t_z]` (m).
    - Sai số `RMSE`.
    - Góc Euler `roll, pitch, yaw` (rad) và dòng `<origin ...>` để copy sang URDF.
- `cam_lidar_calib_recorder.py`
  - **Node ROS 2** giúp **ghi lại các cặp điểm mốc** `(p_L^(i), p_C^(i))` từ 2 topic:
    - `geometry_msgs/PointStamped` trong hệ camera.
    - `geometry_msgs/PointStamped` trong hệ LiDAR.

### Node hỗ trợ checkerboard (camera + LiDAR)

- `cam_checkerboard_point_node.py`
  - Node ROS 2 đọc **ảnh màu + depth + camera_info** từ D435i.
  - Dùng OpenCV tìm **checkerboard** trong ảnh, đọc depth tại tâm checkerboard, back‑project ra 3D.
  - Xuất `geometry_msgs/PointStamped` (3D point) trên topic:
    - Mặc định: `/camera_marker_point`.
  - Frame ID dùng chính frame của depth image (thường là `camera_depth_optical_frame`).

- `lidar_checkerboard_point_node.py`
  - Node ROS 2 đọc `sensor_msgs/LaserScan` từ LiDAR 2D (C1M1).
  - Chọn **một sector góc nhỏ** nơi bạn đặt checkerboard, lọc các tia trong sector + khoảng cách hợp lệ.
  - Chuyển sang toạ độ (x, y) trong hệ `lidar_link`, lấy **trung bình** → tâm checkerboard.
  - Xuất `geometry_msgs/PointStamped` trên topic:
    - Mặc định: `/lidar_marker_point`.

## 2. Yêu cầu môi trường

- ROS 2 **Humble**.
- Đã cài đặt:
  - `opencv-python`
  - `cv_bridge`
- D435i chạy với driver `realsense2_camera` (hoặc tương đương) cung cấp:
  - Ảnh màu (RGB).
  - Depth align với màu.
  - `sensor_msgs/CameraInfo` tương ứng.
- LiDAR 2D C1M1 (hoặc tương đương) cung cấp `sensor_msgs/LaserScan` (ví dụ topic `/scan`).

## 3. Node camera – `cam_checkerboard_point_node.py`

Node này tìm **checkerboard** trên ảnh màu, đọc depth xung quanh tâm checkerboard, back‑project ra 3D.

### Tham số quan trọng

Trong `__init__` của `CamCheckerboardPointNode`:

- `rgb_topic` *(string)*
  - Topic ảnh màu từ D435i.
  - Mặc định: `/camera/color/image_raw`.
- `depth_topic` *(string)*
  - Topic depth **align với ảnh màu**.
  - Mặc định: `/camera/aligned_depth_to_color/image_raw`.
- `camera_info_topic` *(string)*
  - Topic `sensor_msgs/CameraInfo` tương ứng với ảnh màu/depth.
  - Mặc định: `/camera/color/camera_info`.
- `output_topic` *(string)*
  - Topic xuất `geometry_msgs/PointStamped`.
  - Mặc định: `/camera_marker_point`.

Checkerboard:

- `pattern_cols`, `pattern_rows` *(int)*
  - **Số GÓC TRONG (inner corners)** theo chuẩn OpenCV, không phải số ô.
  - Ví dụ: in bàn cờ 7 x 5 ô → inner corners = 6 x 4:
    - `pattern_cols = 6`, `pattern_rows = 4`.
- `square_size` *(float, m)*
  - Kích thước 1 ô (m). Không bắt buộc cho việc tìm tâm, chủ yếu dùng nếu muốn debug sâu.

Depth & encoding:

- `depth_encoding_type` *(string)*
  - `"auto"` – tự đoán (thường đủ với D435i).
  - `"16UC1_mm"` – depth kiểu `uint16`, đơn vị **mm** (realSense thường là kiểu này).
  - `"32FC1_m"` – depth kiểu `float32`, đơn vị **m**.

Debug:

- `publish_debug_image` *(bool)* – xuất ảnh debug có vẽ góc checkerboard.
- `debug_image_topic` *(string)* – topic ảnh debug.
- `show_debug_window` *(bool)* – bật cửa sổ OpenCV (chỉ nên bật nếu bạn chạy node trên máy có màn hình).

### Cách chạy node camera (ví dụ Humble)

Giả sử đang ở thư mục package `my_robot`:

```bash
cd My-Robot-main/my_robot

ros2 run my_robot cam_checkerboard_point_node \  # nếu thêm vào entry_points
  --ros-args \
  -p rgb_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p pattern_cols:=6 -p pattern_rows:=4
```

> Nếu chưa khai node này trong `setup.py`, có thể chạy trực tiếp bằng Python:
>
> ```bash
> python -m my_robot.perception.cam_checkerboard_point_node --ros-args ...
> ```

## 4. Node LiDAR – `lidar_checkerboard_point_node.py`

Node này trích tâm cụm điểm checkerboard từ `LaserScan` trong một sector góc.

### Tham số quan trọng

- `scan_topic` *(string)*
  - Topic `sensor_msgs/LaserScan`.
  - Mặc định: `/scan`.
- `output_topic` *(string)*
  - Topic xuất `geometry_msgs/PointStamped`.
  - Mặc định: `/lidar_marker_point`.

Sector góc:

- `angle_min_deg`, `angle_max_deg` *(float, độ)*
  - Sector góc (độ) trong hệ LiDAR:
    - `0°`  → trục X dương của LiDAR (thường là phía trước robot).
    - `+90°` → quay ngược chiều kim đồng hồ (trục Y dương).
  - Mặc định: `[-20°, 20°]` (một hình quạt hẹp phía trước).
  - Dùng RViz để nhìn LaserScan và điều chỉnh sao cho sector luôn bao được bảng.

Khoảng cách & số tia:

- `min_range`, `max_range` *(float, m)*
  - Khoảng cách hợp lệ đến bảng.
  - Mặc định: `[0.1, 10.0]`.
- `min_points` *(int)*
  - Số tia tối thiểu trong sector để chấp nhận cụm.
  - Nếu quá nhỏ, dễ bị nhiễu; nếu quá lớn mà bảng nhỏ, có thể không bao giờ đủ.

Frame:

- `frame_id_override` *(string)*
  - Nếu rỗng (`""`) → dùng `frame_id` từ LaserScan (thường là `lidar_link`).
  - Nếu TF dùng tên khác (ví dụ `c1m1_link`), có thể đặt tên đó vào đây.

### Cách chạy node LiDAR (ví dụ Humble)

```bash
cd My-Robot-main/my_robot

ros2 run my_robot lidar_checkerboard_point_node \  # nếu thêm vào entry_points
  --ros-args \
  -p scan_topic:=/scan \
  -p angle_min_deg:=-20.0 -p angle_max_deg:=20.0 \
  -p min_range:=0.3 -p max_range:=3.0
```

Hoặc chạy trực tiếp bằng Python:

```bash
python -m my_robot.perception.lidar_checkerboard_point_node --ros-args \
  -p scan_topic:=/scan \
  -p angle_min_deg:=-20.0 -p angle_max_deg:=20.0
```

> Gợi ý: mở RViz, hiển thị `LaserScan` và `PointStamped` để xem tâm cụm điểm có bám đúng bảng không.

## 5. Node ghi dữ liệu – `cam_lidar_calib_recorder.py`

Node này **không tự tìm checkerboard**, mà **chỉ nhận 2 topic PointStamped**:

- `cam_topic` *(string)* – mặc định `/camera_marker_point`.
- `lidar_topic` *(string)* – mặc định `/lidar_marker_point`.

Mỗi lần nhấn `Enter` trong terminal của node này, nó sẽ:

1. Lấy **giá trị mới nhất** của `cam_topic` → `p_C^(i)`.
2. Lấy **giá trị mới nhất** của `lidar_topic` → `p_L^(i)`.
3. Lưu cặp `(p_L^(i), p_C^(i))` vào bộ nhớ.

Khi gõ `q` rồi Enter:

- Node sẽ lưu ra thư mục `out_dir` các file:
  - `{prefix}_points_L.npy`, `{prefix}_points_C.npy`
  - Và bản `.csv` tương ứng.

### Cách chạy recorder (Humble)

```bash
cd My-Robot-main/my_robot

python -m my_robot.perception.cam_lidar_calib_recorder --ros-args \
  -p cam_topic:=/camera_marker_point \
  -p lidar_topic:=/lidar_marker_point \
  -p out_dir:=calib_data \
  -p prefix:=checker1
```

Quy trình thao tác:

1. Đặt checkerboard ở **vị trí 1** (trước robot, trong khung nhìn camera + sector LiDAR).
2. Đợi vài giây để hai topic ổn định.
3. Trong terminal của `cam_lidar_calib_recorder.py` → nhấn `Enter` để ghi 1 mẫu.
4. Di chuyển checkerboard sang **vị trí 2, 3, 4…** (xa–gần, trái–phải, cao–thấp).
5. Mỗi vị trí → nhấn `Enter` một lần.
6. Sau khi có đủ mẫu (nên ≥ 10–20), gõ `q` + Enter.

Kết quả trong `out_dir` (vd. `calib_data`):

- `checker1_points_L.npy` – các điểm trong hệ LiDAR.
- `checker1_points_C.npy` – các điểm tương ứng trong hệ camera.

## 6. Tính R, t bằng Kabsch – `cam_lidar_calib_from_data.py`

Sau khi đã có các file `points_L.npy` và `points_C.npy`, dùng script:

```bash
cd My-Robot-main/my_robot

python -m my_robot.perception.cam_lidar_calib_from_data \
  --points-l calib_data/checker1_points_L.npy \
  --points-c calib_data/checker1_points_C.npy
```

Script sẽ in ra:

- Ma trận quay `R` (3x3).
- Vector tịnh tiến `t = [t_x, t_y, t_z]` (m).
- Khoảng cách tâm–tâm `||t||` (m).
- Sai số trung bình `RMSE` (m).
- Góc Euler `roll, pitch, yaw` (rad) theo chuẩn ROS (Z-Y-X).
- Một dòng URDF dạng:

```xml
<origin xyz="t_x t_y t_z" rpy="roll pitch yaw"/>
```

Sau đó chỉ cần copy dòng này sang file URDF/Xacro (joint giữa camera và LiDAR).

## 7. Ghi kết quả vào URDF

Ví dụ file Xacro chứa joint giữa camera và LiDAR:

```xml
<joint name="lidar_mount" type="fixed">
  <parent link="camera_link"/>
  <child  link="lidar_link"/>
  <origin xyz="..." rpy="..."/>
</joint>
```

- Thay `xyz` và `rpy` bằng giá trị mà script `cam_lidar_calib_from_data.py` in ra.
- Đảm bảo `parent`/`child` trùng khớp với frame dùng trong TF (frame của camera depth / frame của LiDAR).

Sau khi chỉnh URDF, có thể:

- Reload mô hình robot và kiểm tra TF bằng `ros2 run tf2_tools view_frames` hoặc RViz.
- Overlay point cloud/laser lên ảnh camera để xem độ khớp.

---


