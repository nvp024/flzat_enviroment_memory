# FLZAT Environment Memory

ROS 2 Jazzy workspace xây dựng bộ nhớ ngữ nghĩa cho robot trong Gazebo và dùng
bộ nhớ đó để tìm kiếm, trả lời và điều hướng tới vật thể.

## Trạng thái hiện tại

**Toàn bộ flow Mode 1 → tạo RAG → Mode 2 chưa được kiểm thử end-to-end.** Step 2
đã tích hợp VLM grounding vào code, nhưng chưa có replay RGB-D thật để nghiệm
thu tọa độ với ground truth SDF.

| Hạng mục | Trạng thái |
|---|---|
| Mode 1: frontier exploration + SLAM + lưu map | PASS riêng lẻ |
| RGB-D, CameraInfo và TF | PASS smoke riêng lẻ |
| Qwen3-VL-2B → label + bbox Gazebo | PASS offline, 11 ảnh |
| VLM bbox → depth + TF → memory | Đã implement; PASS unit/build, chưa replay thật |
| Mode 2: load map + AMCL + Nav2 | PASS riêng lẻ |
| Truy xuất Chroma và điều hướng bằng text command | PASS với RAG seeded |
| Mode 1 → RAG thật → Mode 2 | Chưa hoàn thiện |
| YOLO legacy | Đã xóa; Mode 1 chỉ còn VLM grounding |

Mode 1 cũ đã quét và lưu map nhưng chưa tạo được RAG thật. Mode 2 đã điều hướng thành công tới
`bench`, nhưng dùng fixture `hotel_demo_14_seeded`; sáu object này không phải
output của lần chạy Mode 1.

## Hai chế độ vận hành

```text
Mode 1
Gazebo → SLAM/Nav2 → frontier exploration → frozen RGB-D/TF observation
→ Qwen3-VL label + bbox → depth + TF → minimal record → Chroma + saved map

Mode 2
Saved map + AMCL → text/speech command → embedding query → Top-K
→ safe approach pose → Nav2 NavigateToPose
```

The package has exactly two public scenario entry points:

- `autonomous_memory_build.launch.py`: quét map và ghi memory.
- `memory_assistant.launch.py`: đọc memory và điều hướng; không sửa database.

## Cấu trúc package

```text
environment_memory/
├── exploration/   # Mode 1, frontier và map saving
├── perception/    # VLM grounding, RGB-D, depth và TF
├── storage/       # record, dedup, embedding, Chroma, manifest
├── retrieval/     # semantic query và CLI
└── assistant/     # command, approach pose và Nav2
```

## Môi trường Ubuntu

Máy mới Ubuntu 24.04 x86_64: chép/clone mã nguồn
`flzat_enviroment_memory` vào thư mục dự định chứa cả ba workspace, rồi chạy
**một lệnh** từ thư mục cha đó:

```bash
bash flzat_enviroment_memory/tools/setup_new_machine.sh
```

Script tự cài ROS 2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox, Miniconda
`py312`, thư viện Python/audio, Git LFS, nguồn frontier-exploration, tải model mặc định
và build cả ba workspace. Chạy bằng user thường; script sẽ hỏi `sudo` khi cài
gói hệ thống. Nếu chỉ có repo memory, script tự clone hai repo còn thiếu.
Chỉ chuyển **mã nguồn**, không chuyển `build/`, `install/`, `log/` từ máy cũ;
script sẽ dừng nếu thấy các thư mục build cũ. Cần mạng và dung lượng đáng kể
để tải các model. Driver NVIDIA/CUDA phụ thuộc GPU máy mới và cần kiểm tra
riêng. Map, ảnh và Chroma dưới `~/.local/share/flzat/environment_memory/`
không nằm trong Git, muốn dùng lại phải chép riêng. Có thể bỏ qua tải model
ban đầu bằng `FLZAT_PREFETCH_MODELS=0 bash flzat_enviroment_memory/tools/setup_new_machine.sh`.
Script dùng NumPy 2.4.6 giống môi trường `py312` hiện tại; không chạy thêm
`pip install -r requirements-memory.txt` vì file đó còn ghim NumPy 1.26.4.

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
  grounding_action_timeout_s:=300.0
```

Nếu không truyền `map_id`, launch tự sinh UUID. Artifact mặc định được lưu tại:

```text
~/.local/share/flzat/environment_memory/<environment_id>/
├── chroma/
├── images/
├── maps/
├── observations.jsonl
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
- VLM chỉ trả label và bbox chuẩn hóa; manager thêm ID/confidence bảo thủ và
  chuyển bbox về pixel.
- Depth, CameraInfo và TF tại timestamp chụp chịu trách nhiệm tính `(x,y,z)`
  trong frame `map`; VLM không được tự sinh tọa độ metric.
- Mode 1 chỉ dùng `/vlm/ground_objects`; YOLO, Ultralytics và semantic
  enrichment trung gian đã được loại khỏi source và launch.
- Runtime Qwen3-VL cần `torch`, `transformers` có class
  `Qwen3VLForConditionalGeneration` và `qwen_vl_utils` trong Conda `py312`.
- NumPy 2 trong Conda dùng bộ chuyển ROS Image thuần NumPy, không dùng Python
  `cv_bridge` đã compile với NumPy 1.x.

Xem [kế hoạch](docs/PLANNING.md) và [kết quả Ubuntu](docs/UBUNTU_TEST_LOG.md).
