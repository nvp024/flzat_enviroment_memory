# Nhật ký kiểm thử Ubuntu

Cập nhật: 2026-09-01. Trạng thái dùng trong tài liệu: `PASS`, `FAIL`,
`PARTIAL`, `BLOCKED`, `NOT_RUN`.

## Tổng quan

| Hạng mục | Trạng thái | Kết luận |
|---|---|---|
| Build ba workspace | PASS | ROS 2 Jazzy, Conda `py312` |
| Unit test Environment Memory | PASS | 97 direct pytest tại lần kiểm tra NumPy 2 |
| RGB-D/TF smoke | PASS | Camera, depth, scan, odom và TF hoạt động |
| Mode 1 exploration + SLAM | PASS | Tự quét và lưu map `hotel_demo_14` |
| Mode 1 semantic memory | PARTIAL | Map thành công nhưng object thật bằng 0 |
| Mode 2 saved-map localization | PASS | AMCL tự initial pose `(0,0,0)` |
| Mode 2 retrieval + text navigation | PASS | Dùng fixture có 6 object seeded |
| Mode 1 → Mode 2 không seeded | NOT_RUN | Chưa có semantic object thật để truy xuất |

## Các lần kiểm thử chính

### 2026-08-28 — build và integration smoke

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.
- OpenArm, FLZAT Robot và Environment Memory build thành công.
- Camera RGB khoảng 10 Hz, depth 7–9 Hz, LiDAR khoảng 10 Hz.
- Các topic `/odom`, `/tf`, `/tf_static`, `/cmd_vel_safe` và topic debug
  memory xuất hiện.
- Direct tests đã pass; cấu hình colcon khi đó còn báo `NO TESTS RAN` cho một
  số Python package.
- Full autonomous scenario chưa chạy ở session này.

### 2026-08-29 — sửa tương thích NumPy 2

Lỗi:

```text
cv_bridge được build với NumPy 1.x nhưng process chạy NumPy 2.4.6.
```

Đã thay conversion Python `cv_bridge` bằng converter ROS Image thuần NumPy cho
`rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8` và `32FC1`.

Kết quả:

```text
Targeted image tests: 12 passed
Full environment-memory direct pytest: 97 passed
Conda py312 rebuild: PASS
```

### 2026-08-31 — Mode 1

- Frontier exploration điều khiển robot quét map tự động.
- SLAM và map saving hoàn tất.
- Artifact map:

```text
/home/phucnv/.local/share/flzat/environment_memory/hotel_demo_14/maps/
```

- `hotel_demo_14` không có semantic object hợp lệ (`objects=0`).
- Kết luận: mapping PASS; YOLO/VLM/persistence semantic chưa đạt E2E.

### 2026-08-31 — Mode 2 seeded fixture

```text
Environment: hotel_demo_14_seeded
Map ID: a8228e3b-eb9d-467c-8230-006684dfbbec
Object count: 6
```

Đã sửa literal Nav2 `use_composition="false"` thành `"False"`. AMCL được cấu
hình tự initial pose tại vị trí spawn `(0,0,0)`; localization, global costmap
và navigation lifecycle đều active.

Kết quả điều hướng:

```text
Command: go to the bench
Object position: (2.20, 1.25, 0.175), frame map
Safe approach: (1.35, 0.40), frame map
Nav2 result: Goal succeeded
Final AMCL pose: (1.281, 0.361), frame map
```

Evidence:

```text
/home/phucnv/.local/state/flzat/environment_memory/logs/
  hotel_demo_14_seeded_mode2/20260831_clean_02/command.txt
/home/phucnv/.local/state/flzat/environment_memory/logs/
  hotel_demo_14_seeded_mode2/20260831_clean_02/terminal.log
```

RViz có cảnh báo GLSL sampler nhưng không chặn map, costmap, AMCL hoặc Nav2.

## Fixture Mode 2

Sáu object seeded, đều dùng frame `map`:

| Object | x | y | z |
|---|---:|---:|---:|
| bench | 2.20 | 1.25 | 0.175 |
| chest | 5.00 | 4.25 | 0.160 |
| fan | 5.00 | 7.30 | 0.425 |
| basket | 6.80 | 4.30 | 0.140 |
| vase | 8.70 | 6.65 | 0.475 |
| plant | 6.00 | -1.40 | 0.210 |

## Lệnh chạy có lưu log

### Mode 1

```bash
RUN_ENV_ID="hotel_demo_15"
RUN_DIR="$HOME/.local/state/flzat/environment_memory/logs/${RUN_ENV_ID}/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR/ros"
export ROS_LOG_DIR="$RUN_DIR/ros"
export RCUTILS_COLORIZED_OUTPUT=0
set -o pipefail

ros2 launch environment_memory autonomous_memory_build.launch.py \
  environment_id:="$RUN_ENV_ID" \
  headless:=false use_rviz:=true use_sim_time:=true \
  semantic_action_timeout_s:=300.0 \
  2>&1 | tee "$RUN_DIR/terminal.log"
```

### Mode 2

```bash
MODE2_ENV_ID="hotel_demo_14_seeded"
MODE2_MAP_ID="a8228e3b-eb9d-467c-8230-006684dfbbec"
MODE2_RUN_DIR="$HOME/.local/state/flzat/environment_memory/logs/${MODE2_ENV_ID}_mode2/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MODE2_RUN_DIR/ros"
export ROS_LOG_DIR="$MODE2_RUN_DIR/ros"
export RCUTILS_COLORIZED_OUTPUT=0
set -o pipefail

MODE2_COMMAND=(
  ros2 launch environment_memory memory_assistant.launch.py
  environment_id:="$MODE2_ENV_ID"
  map_id:="$MODE2_MAP_ID"
  storage_root:="$HOME/.local/share/flzat/environment_memory"
  headless:=false use_rviz:=true use_sim_time:=true
)

printf '%q ' "${MODE2_COMMAND[@]}" | tee "$MODE2_RUN_DIR/command.txt"
printf '\nLog directory: %s\n' "$MODE2_RUN_DIR" | tee -a "$MODE2_RUN_DIR/command.txt"
"${MODE2_COMMAND[@]}" 2>&1 | tee "$MODE2_RUN_DIR/terminal.log"
```

Trước khi chạy, source theo thứ tự:

```bash
conda activate py312
source /opt/ros/jazzy/setup.bash
source <project-root>/openarm_skeleton_v1.2_ws/install/setup.bash
source <project-root>/flzat_robot_ws/install/setup.bash
source <project-root>/flzat_enviroment_memory/install/setup.bash
```

## Việc chưa kiểm thử

- Một object do detector/VLM tạo thật được lưu và truy xuất ở Mode 2.
- Độ chính xác bbox, depth và tọa độ `map` so với ground truth.
- Deduplication runtime qua nhiều góc nhìn.
- Ambiguous/low-score command phải không làm robot di chuyển.
- Hash/count Chroma không đổi sau Mode 2.
- Qwen2-VL-2B và VLM-only bounding box trên Google Colab.

## Mẫu ghi session mới

```text
Date/time:
Commit/dirty files:
Environment ID / Map ID:
Command và log path:
Build/tests: PASS|FAIL
Mode 1 mapping: PASS|FAIL|NOT_RUN
Semantic object count:
Mode 2 retrieval: PASS|FAIL|NOT_RUN
Nav2 result:
Lỗi chính:
Kết luận và bước tiếp theo:
```

## Lịch sử ngắn

| Ngày | Nội dung | Kết quả |
|---|---|---|
| 2026-08-28 | Build + sensor smoke | PARTIAL |
| 2026-08-29 | NumPy 2 converter | PASS |
| 2026-08-31 | Mode 1 SLAM/map | PASS mapping, PARTIAL semantic |
| 2026-08-31 | Mode 2 seeded text navigation | PASS |
