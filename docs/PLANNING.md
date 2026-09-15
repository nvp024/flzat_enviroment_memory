# Kế hoạch Environment Memory V1

Cập nhật: 2026-09-10.

## Mục tiêu

Robot tự quét môi trường, lưu bản đồ và vị trí/ngữ nghĩa của vật thể; sau đó
nhận lệnh tự nhiên, truy xuất vật phù hợp và dùng Nav2 đi tới một pose an toàn
gần vật đó.

Step 1 grounding offline đã pass. Step 2 đã được tích hợp và build nhưng chưa
pass replay RGB-D/TF thật. Chuỗi `Mode 1 → RAG thật → Mode 2` vẫn chưa được xác
nhận runtime.

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
→ Qwen3-VL label + bbox pixel
→ depth localization
→ TF camera → map
→ semantic record tối thiểu
→ deduplication + embedding
→ Chroma + keyframe + map + manifest
```

Nguyên tắc:

- Exploration không chờ VLM; queue giữ một request active và một ảnh pending
  mới nhất.
- VLM xử lý RGB frozen; depth/CameraInfo/TF frozen được giữ tới khi persistence
  trả ACK.
- RGB, depth, CameraInfo và TF phải cùng observation timestamp.
- Tọa độ cuối được lưu trong frame `map`.
- Chỉ record `useful=true` và hợp lệ mới được ghi.
- Finalization chờ grounding, localization và persistence drain.

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
| Public Mode 1/Mode 2 launches | Đã có | Chỉ test riêng từng mode |
| Frontier + SLAM + map saving | Xong | PASS |
| RGB-D + depth + TF geometry | Xong | PASS smoke |
| NumPy 2 image conversion | Xong | PASS |
| Qwen3-VL-2B grounding 11 ảnh | Xong | PASS offline |
| GroundObjects action + parser | Xong | PASS unit/build |
| Frozen bbox → depth/TF → memory | Xong code | Chưa replay RGB-D thật |
| Chroma, embedding, retrieval | Đã có code | PASS unit; Mode 2 seeded PASS |
| AMCL auto initial pose | Xong | PASS |
| Text command → Nav2 | Xong | PASS seeded fixture |
| YOLO legacy | Đã xóa khỏi source/launch | Không còn rollback |
| Mode 1 → RAG thật → Mode 2 | Chưa hoàn thiện | NOT_RUN |

## Vấn đề còn lại

### 1. Nghiệm thu Step 2

Chưa chạy thành công chuỗi với synchronized RGB-D thật:

```text
GroundObjects → bbox → frozen depth/TF → localized object → dedup → Chroma
```

Seeded RAG từ SDF chỉ dùng làm ground-truth oracle; không được ghi đè store đầu
ra VLM. Cần capture/replay RGB, depth, CameraInfo và TF cùng timestamp rồi đo
sai số tới thể tích object SDF, ngưỡng tối đa `0.20 m`.

### 2. Trạng thái model

Qwen3-VL-2B đã đạt gate bbox offline với prompt v4. Máy Ubuntu hiện thiếu
`qwen_vl_utils`, nên inference model thật chưa chạy tại local. Production code
không repair response lỗi; observation sau là cơ hội quan sát lại.

### 3. Retrieval safety

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
