# FLZAT Environment Memory Workspace

Independent ROS 2 Jazzy integration workspace for Version 1 autonomous
environment memory.

The current workspace includes the Phase 9 operating-mode integration:

- workspace and overlay foundation;
- shared interfaces, reusable speech launch, and shared VLM action broker;
- frontier-exploration supervision and SLAM map saving;
- synchronized, event-triggered RGB-D observation acquisition;
- pinned YOLOv8n detection and RGB-D localization in the global `map` frame;
- canonical object records, spatial-semantic deduplication, multilingual
  embeddings, atomic keyframes, embedded Chroma, restart recovery, and
  checksum-bound manifest finalization;
- completed-manifest verification, filtered semantic retrieval, typed voice
  commands, ambiguity clarification, TTS results, safe approach-pose
  generation, Nav2 path validation, and NavigateToPose execution.

The workspace exposes exactly two public scenario entry points:

- `autonomous_memory_build.launch.py`
- `memory_assistant.launch.py`

The Phase 6 semantic manager now publishes detector-linked, map-localized
objects on `/environment_memory/localized_observations`. Implementation through
Phase 9 is present; Phase 10 Ubuntu end-to-end acceptance remains to be
performed.

## Overlay order

```bash
source /opt/ros/jazzy/setup.bash
source <integrate-root>/flzat_nav_ws/install/setup.bash
source <integrate-root>/flzat-voice-ros2/install/setup.bash
source <integrate-root>/flzat_environment_memory_ws/install/setup.bash
```

## Fetch and build on Ubuntu

```bash
cd <integrate-root>/flzat_environment_memory_ws
vcs import < environment_memory.repos
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-detector.txt
python -m pip install -r requirements-memory.txt
python tools/fetch_yolov8n.py
source /opt/ros/jazzy/setup.bash
source <integrate-root>/flzat_nav_ws/install/setup.bash
source <integrate-root>/flzat-voice-ros2/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

The pinned frontier explorer is v1.6.1 commit
`b0fad500e5c81ad3154f0469ca283b2702a3f90c`.
The detector runtime is Ultralytics 8.3.0. The official v8.3.0 YOLOv8n
weights are fetched to the configurable runtime data directory and accepted
only when their SHA-256 is
`f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36`.

## Public mode 1: autonomous memory build

After building and sourcing all overlays:

```bash
ros2 launch environment_memory autonomous_memory_build.launch.py \
  environment_id:=hotel_demo map_id:=<mapping-session-uuid> \
  headless:=true use_rviz:=false
```

This public launch starts the existing Gazebo/SLAM/Nav2/watchdog stack,
the pinned frontier explorer in cold-idle mode, the build manager, and the
observation manager. Triggered observations run YOLOv8n once, filter depth in
the central 60% of each box, and publish accepted geometry on
`/environment_memory/geometric_observations`. Each result contains camera and
`map` points plus the robot pose at the RGB timestamp. The same observation is
published once on `/environment_memory/vlm_observations` with its frozen,
annotated JPEG and all detections that have valid geometry. This VLM image
contains box IDs/classes but deliberately omits the map-coordinate debug text.

`semantic_observation_manager` sends that frozen batch to the shared
`/vlm/analyze_environment` action. The VLM server validates the strict Version
1 schema, allows one repair retry, and rejects a second invalid response. The
manager joins results by both `observation_id` and `detection_id`; useful
objects are then published on `/environment_memory/localized_observations`.
The VLM never supplies geometry or navigation commands.

The launch starts the writable Memory Manager and leaves retrieval and assistant
logic disabled. Useful localized semantic observations are written beneath
`~/.local/share/flzat/environment_memory/<environment_id>/`.

The manager writes `manifest.json` with `status: incomplete` while building.
At frontier completion it stops new writes, flushes Chroma, waits before map
saving, hashes the saved map YAML, and atomically marks the manifest complete.
Restarting an incomplete build with the same environment and map IDs recovers
the existing Chroma records. A completed manifest cannot be reopened writable.

The semantic embedding model is
`sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2`, pinned to model
revision `e8f8c211226b894fcb81acc59f3b34ba3efd5f42`. Only semantic text is
embedded; coordinates, timestamps, IDs, and confidence values remain metadata.

The default model path is
`~/.local/share/flzat/environment_memory/models/yolov8n-v8.3.0.pt`. Override
it with a YAML file passed as `detector_config:=...`; the configured SHA-256
must match the file or the observation manager refuses to start.

Record Ubuntu results in `docs/UBUNTU_TEST_LOG.md`. Windows checks are not ROS
or Gazebo acceptance.

## Public mode 2: memory assistant

Phase 8 adds the `/environment_memory/query` service and `query_memory` CLI.
The query server refuses incomplete manifests, map checksum mismatches, map-ID
mismatches, and database object-count mismatches. Its Chroma adapter exposes
query operations only; assistant mode contains no upsert, add, update, or
delete path.

After the first mode produces a complete manifest:

```bash
ros2 launch environment_memory memory_assistant.launch.py \
  environment_id:=hotel_demo map_id:=<mapping-session-uuid>

ros2 run environment_memory query_memory "blue water bottle" \
  --environment-id hotel_demo --map-id <mapping-session-uuid>
```

The launch refuses an incomplete manifest, map checksum mismatch, missing map,
wrong environment ID, or wrong mapping-session ID before starting the scenario.
It obtains the saved map path from the verified manifest, starts Gazebo and
Nav2 with `slam:=false`, and then starts the internal assistant runtime.

That runtime reuses `speech_services.launch.py` rather than
the audio loopback pipeline. It also loads the existing shared VLM runtime as
planned, but Version 1 intent and retrieval decisions are deterministic. The
VLM is never allowed to provide numeric coordinates or send navigation goals.

For explicit navigation, the command manager filters 0.8, 1.0, and 1.2 metre
standoff poses against both `/map` and `/global_costmap/costmap`, validates
candidate paths through `/compute_path_to_pose`, and sends only the nearest
valid approach pose to `/navigate_to_pose`. Ambiguous high-scoring matches
produce a spoken clarification and no movement.

## Required and common launch arguments

`autonomous_memory_build.launch.py`:

- `environment_id`: persistent environment directory name; default
  `hotel_demo`.
- `map_id`: unique mapping-session ID. A UUID is generated by default; pass the
  previous value only when resuming the same incomplete build.
- `storage_root`: optional persistence root.
- `headless`, `use_rviz`, `transport_partition`: simulator controls.
- `detector_config`, `embedding_device`, `vlm_device`: model/runtime controls.
- `readiness_timeout_s`, `semantic_action_timeout_s`,
  `finalization_timeout_s`: bounded startup and drain controls.

`memory_assistant.launch.py`:

- `environment_id`: completed environment to open.
- `map_id`: optional expected ID; leaving it empty uses the verified manifest.
- `storage_root`: must identify the same persistence root used for building.
- `headless`, `use_rviz`, `transport_partition`: simulator controls.
- `whisper_language`, `embedding_device`, `vlm_device`: assistant model controls.

Internal files such as `exploration_observation.launch.py` and
`assistant_runtime.launch.py` remain implementation details used by the two
public entry points. They are not additional supported operating modes.
