# Kế hoạch Environment Memory V1

Cập nhật: 2026-09-01.

## Mục tiêu

Robot tự quét môi trường, lưu bản đồ và vị trí/ngữ nghĩa của vật thể; sau đó
nhận lệnh tự nhiên, truy xuất vật phù hợp và dùng Nav2 đi tới một pose an toàn
gần vật đó.

## Phân chia workspace

| Workspace | Trách nhiệm |
|---|---|
| `openarm_skeleton_v1.2_ws` | Gazebo, robot, RGB-D, LiDAR, TF, SLAM, AMCL, Nav2 |
| `flzat_robot_ws` | VAD, STT, TTS và VLM runtime |
| `flzat_enviroment_memory` | Exploration, perception, memory, retrieval và assistant |

Thứ tự source:

```text
ROS Jazzy → OpenArm → FLZAT Robot → Environment Memory
```

## Kiến trúc Mode 1

```text
Gazebo + RGB-D + LiDAR
→ SLAM + Nav2 + frontier exploration
→ trigger observation
→ RGB/depth/CameraInfo đồng bộ
→ bounding box
→ depth localization
→ TF camera → map
→ VLM semantic JSON
→ deduplication + embedding
→ Chroma + keyframe + map + manifest
```

Nguyên tắc:

- VLM xử lý ảnh frozen, không xử lý video liên tục.
- RGB, depth, CameraInfo và TF phải cùng observation timestamp.
- Tọa độ cuối được lưu trong frame `map`.
- Chỉ record `useful=true` và hợp lệ mới được ghi.
- Finalization chờ semantic queue hoàn tất rồi mới đánh dấu manifest complete.

## Kiến trúc Mode 2

```text
Completed manifest
→ saved map + AMCL + Nav2
→ text/STT command
→ parse intent
→ semantic embedding query, Top-K tối đa 5
→ xử lý mơ hồ
→ tạo pose cách vật 0.8–1.2 m
→ kiểm tra map/costmap/path
→ NavigateToPose
```

Mode 2 mở Chroma read-only. VLM không cung cấp tọa độ hay gửi Nav2 goal trong
Version 1.

## Dữ liệu lưu trữ

Mỗi object gồm:

- label, description, attributes, relationships và scene;
- tọa độ `map`, robot pose và timestamps;
- detector/semantic/localization confidence;
- `seen_count` và keyframe path;
- vector embedding và canonical `record_json` trong Chroma metadata.

Text dùng để embedding chỉ chứa semantic. ID, tọa độ và confidence là metadata.

## Trạng thái thực hiện

| Phần | Source | Runtime Ubuntu |
|---|---:|---:|
| Public Mode 1/Mode 2 launches | Xong | PASS một phần/E2E theo bảng dưới |
| Frontier + SLAM + map saving | Xong | PASS |
| RGB-D + depth + TF geometry | Xong | PASS smoke |
| NumPy 2 image conversion | Xong | PASS |
| Chroma, embedding, retrieval | Xong | PASS unit; Mode 2 seeded PASS |
| AMCL auto initial pose | Xong | PASS |
| Text command → Nav2 | Xong | PASS seeded fixture |
| YOLO → VLM → real memory record | Xong về code | Chưa đạt E2E |

## Vấn đề còn lại

### 1. Semantic pipeline của Mode 1

Mode 1 đã quét và lưu map, nhưng chưa chứng minh được chuỗi:

```text
detector → VLM JSON → localized object → dedup → Chroma
```

YOLOv8n COCO khó nhận các model hình khối trong Gazebo. SmolVLM2-500M cũng có
hạn chế về nhận dạng và tuân thủ JSON.

### 2. Thử nghiệm VLM trên Colab

So sánh SmolVLM2 và Qwen2-VL-2B bằng ảnh camera Gazebo. Phải dùng schema
`environment_memory.v1`, không dùng prompt `navigation_json` của companion.

Đo tối thiểu:

- JSON hợp lệ ở lần đầu và sau một repair retry;
- label/scene đúng, không hallucinate;
- detection ID hoặc bounding box đúng;
- latency warm/cold, VRAM và tỷ lệ thành công.

### 3. Hướng VLM-only đang nghiên cứu

VLM có thể thay detector 2D nếu trả được bounding box chuẩn hóa:

```json
{"label":"standing_fan","bbox":[0.32,0.15,0.57,0.91],"confidence":0.91}
```

Sau đó hệ thống vẫn dùng depth, CameraInfo và TF để tính vị trí trong `map`.
Hướng này chưa implement. Cần kiểm tra độ chính xác bbox trước khi thay YOLO;
giữ detector abstraction để có thể chọn YOLO, VLM grounding hoặc Gazebo
ground truth.

### 4. Retrieval safety

Cần bổ sung log Top-K cho đúng lần `/text_command` và đánh giá ngưỡng điểm tối
thiểu. Hiện hệ thống xử lý ambiguity theo score margin nhưng chưa có acceptance
threshold được xác nhận bằng dữ liệu thực.

## Tiêu chí hoàn thành V1

- Mode 1 tự quét, lưu completed manifest và ít nhất một object không seeded.
- Object có tọa độ `map` hợp lệ từ depth + TF.
- Quan sát lặp lại được deduplicate đúng.
- Restart vẫn đọc được map, Chroma và keyframe.
- Query đúng object trong Top-3.
- Mode 2 đi tới pose an toàn, không chạy vào tâm vật.
- Object mơ hồ/không đủ tin cậy không làm robot di chuyển.
- Có log Ubuntu tái lập được cho toàn bộ chuỗi Mode 1 → Mode 2.

## Ngoài phạm vi V1

Multi-floor, lifelong memory, tracking động dài hạn, dense 3D reconstruction,
cloud VLM production và điều khiển robot trực tiếp bằng output VLM được để cho
phiên bản sau.
