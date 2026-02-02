# my_robot_nav

Gói điều hướng (navigation) cho robot thật, dùng **RTAB‑Map** làm localization và **Nav2** để lập kế hoạch đường đi / tránh vật cản. Thư mục này chỉ tập trung cho việc “click trên map → robot tự chạy”, các thứ khác (SLAM, YOLO, NLP…) nằm ở package khác.

---

## Cách chạy

### 1. Mapping (tạo map lần đầu – dùng package my_robot)

```bash
cd ~/ros2_ws
source install/setup.bash

# Tạo map bằng LiDAR + camera, lưu vào ~/ros2_ws/maps/my_home.db
ros2 launch my_robot full_mapping.launch.py
```

Điều khiển robot đi khắp khu vực (teleop hoặc điều khiển riêng), đến khi map đủ thì Ctrl+C. File DB được dùng lại ở bước navigation.

### 2. Navigation (dùng lại map để điều khiển trên RViz)

```bash
cd ~/ros2_ws
source install/setup.bash

# Dùng DB mặc định ~/ros2_ws/maps/my_home.db
ros2 launch my_robot_nav navigation.launch.py

# Hoặc chỉ rõ DB khác
ros2 launch my_robot_nav navigation.launch.py map:=/path/to/your_map.db
```

Trong RViz (config đi kèm Nav2):

1. Fixed Frame = `map`.
2. Thấy map 2D, robot model và costmap.
3. Chọn công cụ `Nav2 Goal` / `2D Goal Pose`, click điểm trên map → Nav2 tính đường và publish `/cmd_vel` để robot chạy đến đó.

---

## Tác dụng từng thư mục / file

```
my_robot_nav/
├── config/
│   └── nav2_params.yaml
│       Tham số Nav2 (bt_navigator, planner, controller, costmap, v.v.).
│       - Dùng frame `map` / `odom` / `base_link` khớp với my_robot.
│       - Local/global costmap đọc dữ liệu từ LiDAR `/scan`.
│
├── launch/
│   ├── localization.launch.py
│   │   - Khởi động RealSense + LiDAR từ package my_robot.
│   │   - Chạy RTAB‑Map ở chế độ localization‑only:
│   │       * load database map (.db) từ tham số `map`.
│   │       * xuất TF `map -> odom` và topic `/map` cho Nav2.
│   │   - Publish static TF `base_link ↔ camera_link`, `base_link ↔ lidar_link`
│   │     nếu không dùng robot_state_publisher riêng.
│   │
│   └── navigation.launch.py
│       - Gọi `localization.launch.py` ở trên.
│       - Gọi `nav2_bringup/navigation_launch.py` với file config
│         `config/nav2_params.yaml`.
│       - Đây là launch chính để điều khiển robot trực tiếp trên map.
│
├── package.xml / setup.py / setup.cfg
│   - Khai báo package ROS2 chuẩn (cài đặt, phụ thuộc Nav2, rtabmap).
│
└── README.md
    - File bạn đang đọc.
```

---

## Ghi chú sử dụng nhanh

- Cần cài các gói:
  - `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-rtabmap-ros`.
- Tham số Nav2 chỉnh trong [my_robot_nav/config/nav2_params.yaml](my_robot_nav/config/nav2_params.yaml):
  - `max_vel_x`, `max_vel_theta` – tốc độ.
  - `robot_radius`, `inflation_radius` – kích thước và vùng an toàn.
  - `xy_goal_tolerance`, `yaw_goal_tolerance` – sai số chấp nhận khi tới đích.
- Khi Nav2 không đi được:
  - Kiểm tra TF: `ros2 run tf2_tools view_frames`.
  - Kiểm tra `/map`, `/scan`, `/odom` có dữ liệu.
  - Kiểm tra `/cmd_vel` có được publish khi đặt goal.

