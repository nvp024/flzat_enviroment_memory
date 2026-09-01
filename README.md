# FLZAT Environment Memory

ROS 2 Jazzy workspace xây dựng bộ nhớ ngữ nghĩa cho robot trong Gazebo và dùng
bộ nhớ đó để tìm kiếm, trả lời và điều hướng tới vật thể.

## Trạng thái hiện tại

| Hạng mục | Trạng thái |
|---|---|
| Mode 1: frontier exploration + SLAM + lưu map | PASS |
| RGB-D, CameraInfo và TF | PASS |
| Mode 2: load map + AMCL + Nav2 | PASS |
| Truy xuất Chroma và điều hướng bằng text command | PASS với dữ liệu seeded |
| YOLO → VLM → semantic record → Chroma | Chưa xác nhận end-to-end |
| VLM phát hiện trực tiếp bounding box | Đang nghiên cứu, chưa implement |

Mode 2 đã điều hướng thành công tới `bench` trong fixture
`hotel_demo_14_seeded`. Mode 1 đã quét và lưu map, nhưng lần chạy
`hotel_demo_14` không tạo object thật; sáu object dùng để kiểm thử Mode 2 là dữ
liệu seeded.

## Hai chế độ vận hành

```text
Mode 1
Gazebo → SLAM/Nav2 → frontier exploration → RGB-D observation
→ detector → depth + TF → VLM semantics → embedding → Chroma + saved map

Mode 2
Saved map + AMCL → text/speech command → embedding query → Top-K
→ safe approach pose → Nav2 NavigateToPose
```

Hai public launch file:

- `autonomous_memory_build.launch.py`: quét map và ghi memory.
- `memory_assistant.launch.py`: đọc memory và điều hướng; không sửa database.

## Cấu trúc package

```text
environment_memory/
├── exploration/   # Mode 1, frontier và map saving
├── perception/    # RGB-D, detector, depth và TF
├── semantics/     # VLM batch và semantic result
├── storage/       # record, dedup, embedding, Chroma, manifest
├── retrieval/     # semantic query và CLI
└── assistant/     # command, approach pose và Nav2
```

## Môi trường Ubuntu

Runtime đã kiểm thử dùng Conda `py312`:

```bash
conda activate py312
source /opt/ros/jazzy/setup.bash
source <project-root>/openarm_skeleton_v1.2_ws/install/setup.bash
source <project-root>/flzat_robot_ws/install/setup.bash
source <project-root>/flzat_enviroment_memory/install/setup.bash
```

Build workspace:

```bash
cd <project-root>/flzat_enviroment_memory
colcon build --symlink-install
source install/setup.bash
```

## Chạy Mode 1

```bash
ros2 launch environment_memory autonomous_memory_build.launch.py \
  environment_id:=hotel_demo \
  headless:=false \
  use_rviz:=true \
  semantic_action_timeout_s:=300.0
```

Nếu không truyền `map_id`, launch tự sinh UUID. Artifact mặc định được lưu tại:

```text
~/.local/share/flzat/environment_memory/<environment_id>/
├── chroma/
├── images/
├── maps/
└── manifest.json
```

Chỉ environment có `manifest.status=complete` mới được Mode 2 mở.

## Chạy Mode 2

```bash
ros2 launch environment_memory memory_assistant.launch.py \
  environment_id:=hotel_demo_14_seeded \
  map_id:=a8228e3b-eb9d-467c-8230-006684dfbbec
```

Không truyền `map_id` thì hệ thống dùng map ID trong completed manifest của
đúng `environment_id`; nó không tự chọn environment mới nhất.

Gửi lệnh không qua VAD/STT:

```bash
ros2 topic pub --once /environment_memory/text_command \
  std_msgs/msg/String "{data: 'go to the fan'}"
```

Kiểm tra Top-K độc lập:

```bash
ros2 run environment_memory query_memory "the fan" \
  --top-k 5 \
  --environment-id hotel_demo_14_seeded \
  --map-id a8228e3b-eb9d-467c-8230-006684dfbbec
```

## Ghi chú kỹ thuật

- Tọa độ object được lưu trong frame `map`, không phải tọa độ tương đối với
  robot.
- Depth + CameraInfo tạo điểm 3D trong camera frame; TF chuyển điểm đó sang
  `map`.
- VLM hiện chỉ bổ sung semantic, không được tự sinh tọa độ metric.
- Chuyển sang VLM-only cần VLM trả bounding box; depth và TF vẫn chịu trách
  nhiệm tính `(x, y, z)`.
- NumPy 2 trong Conda dùng bộ chuyển ROS Image thuần NumPy, không dùng Python
  `cv_bridge` đã compile với NumPy 1.x.

Xem [kế hoạch](docs/PLANNING.md) và [kết quả Ubuntu](docs/UBUNTU_TEST_LOG.md).
