# Version 1 Autonomous Environment Memory — Technical Plan

Canonical planning location after Phase 1 workspace creation:

```text
flzat_environment_memory_ws/docs/PLANNING.md
flzat_environment_memory_ws/docs/UBUNTU_TEST_LOG.md
```

## 1. Goal and workspace ownership

Version 1 is a Gazebo-first, object-centric environment memory system. It
autonomously explores an initially unknown indoor environment, builds a SLAM
map, extracts useful object observations, estimates object positions in the
global `map` frame, and stores persistent semantic records for later retrieval
and navigation.

The design follows the lightweight memory-building and retrieval separation of
[ReMEmbR](https://arxiv.org/abs/2409.13682): semantic descriptions are stored
with position and time. The richer 3D primitives, visual memory, temporal
reasoning, and task-conditioned redundancy reduction described by
[STaR](https://arxiv.org/abs/2602.09255) are references for later versions, not
Version 1 requirements.

Environment memory is an integration layer and must have one owner. It will be
implemented as an independent Git repository and ROS 2 workspace:

```text
Integrate/
├── flzat_nav_ws/                   # Low-level simulation and navigation
├── flzat-voice-ros2/               # High-level speech and shared VLM runtime
└── flzat_environment_memory_ws/    # Environment-memory integration
    ├── docs/
    │   ├── PLANNING.md
    │   └── UBUNTU_TEST_LOG.md
    └── src/
        ├── environment_memory_interfaces/
        ├── environment_memory/
        └── frontier_exploration_ros2/   # Pinned external dependency
```

Build and source order:

```text
flzat_nav_ws
→ flzat-voice-ros2
→ flzat_environment_memory_ws
```

Responsibilities remain separated:

- `flzat_nav_ws` owns Gazebo, robot description, RGB-D, LiDAR, SLAM Toolbox,
  AMCL, Nav2, TF, odometry, costmaps, and the command watchdog.
- `flzat-voice-ros2` owns VAD, STT, TTS, VLM model loading/backends,
  cooperative inference cancellation, and reusable speech/VLM services.
- `flzat_environment_memory_ws` owns frontier-exploration supervision,
  observation triggering, synchronized RGB-D processing, object detection,
  geometric localization, memory management, embedding, persistence,
  retrieval, command interpretation, and the two complete operating modes.

Only narrow public-interface changes are made to the existing repositories.
Memory logic must not be duplicated inside either existing workspace.

### 1.1 Development and test platforms

The initial implementation may be written and statically reviewed from the
current Windows workspace. Windows is the development host, not the runtime
acceptance platform for Version 1.

The implementation must therefore remain portable and Linux-ready:

- Use Python 3, ROS 2 `ament_python`/`ament_cmake`, standard ROS package
  discovery, and `pathlib` instead of hard-coded Windows or Ubuntu paths.
- Resolve package assets through the ROS 2 ament index. Runtime data paths are
  launch arguments or parameters; they are never source-tree-relative
  assumptions.
- Commit text files with LF line endings and UTF-8 encoding. Track executable
  permissions for ROS scripts and helper tools in Git.
- Do not introduce Windows-only APIs, drive-letter paths, PowerShell runtime
  dependencies, or case-insensitive import/file-name assumptions.
- Keep model, Chroma, map, and image locations configurable. The Ubuntu
  default remains `~/.local/share/flzat/environment_memory`.
- Isolate pure Python logic from ROS nodes so geometry, schema, trigger,
  deduplication, and storage tests can run without Gazebo where practical.
- Pin Python and external ROS dependencies in repository manifests rather than
  relying on packages installed manually on the development computer.

The supported Version 1 validation target is Ubuntu with ROS 2 Jazzy. All
`colcon` builds, launch tests, Gazebo/SLAM/Nav2 runs, GPU/model checks, and
end-to-end acceptance results must be produced there. A successful Windows
syntax or unit-test run is useful, but it does not count as ROS acceptance.

Every Ubuntu validation session is recorded in `docs/UBUNTU_TEST_LOG.md`.
Failures must include the command, relevant output, suspected cause, and the
next action so fixes can be prepared on Windows and verified during the next
Ubuntu session.

## 2. Target architecture and operating modes

### 2.1 Memory-building data flow

```text
Gazebo hotel + SLAM Toolbox + Nav2
                 │
                 ├── /map, /scan, /odom, TF
                 ├── /camera/color/image_raw
                 ├── /camera/depth/image_raw
                 └── /camera/camera_info
                              │
Pinned frontier explorer → Memory Build Manager
                              │
                    Observation Manager
       trigger → RGB-D sync → YOLO detections
                              │
            depth localization + timestamped TF2
                              │
               background structured VLM
                              │
                  localized object records
                              │
                     Memory Manager
           deduplicate → embed → persistent Chroma
                              │
                  saved map + DB manifest
```

The robot continues navigating while the VLM processes a frozen observation.
The VLM does not consume a live video stream. Every result is joined back to
the immutable RGB/depth/CameraInfo/TF/pose bundle using `observation_id` and
`detection_id`; it never uses the robot's newer pose when inference completes.

### 2.2 Two public launch files

The third workspace exposes exactly two public scenario entry points. Internal
launch files may be included, but users should not need additional top-level
commands.

#### `autonomous_memory_build.launch.py`

Purpose: explore an unknown environment and build its map and object database.

It launches:

```text
Gazebo + RGB-D + LiDAR
→ SLAM Toolbox (`slam:=true`)
→ Nav2 + watchdog
→ frontier exploration
→ memory_build_manager
→ observation_manager
→ detector and geometry pipeline
→ shared VLM server in environment-analysis mode
→ writable memory_manager
```

It does not launch VAD, STT, TTS, the V4L2 camera node, audio loopback, or the
conversational multimodal manager. Observation generation is autonomous.

#### `memory_assistant.launch.py`

Purpose: load a completed environment, wait for user commands, answer memory
questions, and navigate to remembered objects when explicitly requested.

It launches:

```text
Gazebo
→ saved map + AMCL (`slam:=false`)
→ Nav2 + LiDAR obstacle avoidance + watchdog
→ reusable VAD/STT/TTS services
→ shared VLM server
→ read-only memory_manager
→ memory_command_manager
```

Background exploration and background memory updates are disabled. The RGB-D
topics may remain available, but they are not continuously sent to the VLM or
written to the database.

The assistant launch must not include the current `voice_pipeline.launch.py`
because that launch starts `audio_loopback_node`, which would echo every STT
result directly to TTS and conflict with command handling. The high-level
workspace should instead provide a reusable `speech_services.launch.py` that
starts only VAD, STT, and TTS. The existing `companion_pipeline.launch.py`
remains a separate live companion mode.

## 3. ROS 2 modules, managers, and interfaces

### 3.1 Low-level integration

- Keep the public motion chain unchanged:

  ```text
  Nav2 → /cmd_vel_nav → velocity_smoother → /cmd_vel
       → wall-time watchdog → /cmd_vel_safe → Gazebo
  ```

- Pin the tested commit corresponding to Version 1.6.1 of
  [`frontier_exploration_ros2`](https://github.com/mertgulerx/frontier_exploration_ros2)
  in the third workspace's `.repos` manifest.
- The explorer subscribes to the map/costmaps and dispatches only
  `NavigateToPose` goals. It never publishes velocity commands directly.
- Existing low-level topic, TF, Nav2, and watchdog contracts remain unchanged.

### 3.2 Environment-memory managers

- `memory_build_manager` owns build-session state: readiness, exploration
  start/stop, completion, inference draining, map saving, DB finalization, and
  manifest creation.
- `observation_manager` owns triggers, synchronized sensor bundles, quality
  gates, object detection, depth localization, timestamped TF2, VLM job
  scheduling, and debug images.
- `memory_manager` is the single database owner. It validates localized
  observations, deduplicates objects, generates embeddings, stores keyframes,
  performs atomic upserts, and serves retrieval. It runs writable in build
  mode and read-only in assistant mode.
- `memory_command_manager` owns the assistant state machine: STT transcript,
  typed intent, retrieval, ambiguity handling, spoken response, safe approach
  goal generation, Nav2 action execution, and final TTS status.

### 3.3 Shared VLM runtime

Reuse from `flzat-voice-ros2`:

- SmolVLM2/Qwen2-VL backend registry and model loading.
- Image decoding and prompt profiles.
- Cooperative cancellation.
- One-active/one-latest-pending scheduling concepts.

Do not reuse the V4L2 `camera_node`, its frame-only buffer, or the
conversation-specific multimodal manager for environment memory.

One loaded VLM server handles both conversational and environment-analysis
actions. The global inference priority is:

```text
voice or voice_motion
→ environment_memory
→ motion-only companion event
```

Only one inference runs at a time, with at most one latest pending request.
Voice may cooperatively cancel background environment inference. In the
autonomous build launch no voice nodes are present, so the model is dedicated
to memory observations.

### 3.4 Public interfaces

Add VLM-facing types to `flzat-voice-ros2/robot_interfaces` because the shared
VLM server owns this API:

- `ObjectDetection2D.msg`: observation ID, detection ID, detector class and
  confidence, and pixel bounding box.
- `SemanticObject.msg`: detection ID, semantic label, description, attributes,
  relationships, usefulness flag, and semantic confidence.
- `AnalyzeEnvironment.action`:
  - Goal: observation ID, timestamped compressed RGB image with numbered box
    overlays, and detections.
  - Result: success, scene label, semantic objects, raw-response diagnostic,
    and error.
  - Feedback: queued, inference, validating, retrying, complete.

Add memory-owned types to
`flzat_environment_memory_ws/environment_memory_interfaces`:

- `ExplorationStatus.msg`: state (`WAITING`, `EXPLORING`, `FINALIZING`,
  `COMPLETED`, `FAILED`), goal counts, timestamp, current goal, and reason.
- `LocalizedObjectObservation.msg`: semantic object, detector metadata,
  `PointStamped` in `map`, robot `PoseStamped`, scene, sensor timestamps,
  localization quality, image reference, environment ID, and map ID.
- `MemoryObject.msg`: persisted fields returned by retrieval.
- `QueryMemory.srv`: query text, top-k, environment/map filters, and optional
  scene/time/radius filters; result contains records and cosine scores.

Public endpoints:

```text
/exploration/status
/vlm/analyze_environment
/environment_memory/localized_observations
/environment_memory/status
/environment_memory/debug_image
/environment_memory/query
```

Sensor subscriptions use Best Effort sensor-data QoS. Memory/status
publications and services use Reliable QoS.

## 4. Autonomous observation and localization pipeline

### 4.1 Trigger policy

The build manager starts exploration only after `/map`, `/scan`, Nav2
lifecycle nodes, and the complete
`map → odom → base_footprint → base_link → camera_optical_frame` chain are
available.

An observation becomes eligible when any condition is met:

- A frontier navigation goal completes.
- Translation since the last accepted observation reaches `1.0 m`.
- Accumulated yaw change reaches `45°`.
- HSV histogram scene distance from the last accepted image reaches `0.35`.
- Exploration runs for `20 s` without an accepted observation.
- The first valid synchronized RGB-D/TF bundle becomes available.

Raw frame-to-frame motion alone is not a trigger because ego-motion changes
most pixels while the robot drives.

Rate and load controls:

- Minimum VLM interval: `8 s`.
- Wait `0.75 s` after waypoint arrival for a stable frame.
- Prefer linear speed below `0.10 m/s` and angular speed below `0.15 rad/s`.
- Keep one active observation and one latest pending observation.
- Priority: waypoint, scene change, rotation, translation, timed refresh.
- Replace lower-priority pending work; never build an unbounded queue.

### 4.2 Immutable observation bundle

For every accepted trigger, capture and retain:

```text
observation_id
RGB image and timestamp
depth image and timestamp
CameraInfo
latest scan timestamp/health
map ← camera_optical_frame transform at RGB timestamp
map ← base_link pose at RGB timestamp
trigger reason
```

Approximate-sync RGB and depth with queue size `10` and maximum timestamp
difference `80 ms`. CameraInfo must match resolution/frame and be no older than
`1 s`. RGB/depth and `/scan` health must be no older than `0.5 s`.

### 4.3 Detection and depth localization

- Default detector: pinned YOLOv8n COCO weights.
- Detector confidence: `≥0.35`; NMS IoU: `0.50`; maximum eight detections.
- Ignore `person` by default.
- Run the detector only for triggered observations.
- Send one annotated full RGB image to the VLM, not one request per object.

For each detection:

1. Clamp and validate the bounding box.
2. Use its central 60% for depth association.
3. Accept finite `32FC1` values within `0.20–10.0 m`.
4. Require at least 30 valid samples and valid-depth ratio `≥0.30`.
5. Reject outliers with median absolute deviation.
6. Use the median valid pixel `(u,v)` and median depth `Z`.
7. Back-project with CameraInfo:

   ```text
   X = (u - cx) × Z / fx
   Y = (v - cy) × Z / fy
   Z = median_depth
   ```

8. Create a `PointStamped` in `camera_optical_frame`.
9. Transform it with `map ← camera_optical_frame` at the RGB timestamp.
10. Save `map ← base_link` at the same timestamp as the robot pose.

Use a `0.5 s` TF lookup timeout. Reject invalid depth, intrinsics, frame IDs,
or timestamped transforms; never substitute the latest transform.

Localization quality combines valid-depth ratio and normalized depth
dispersion. Overall confidence is the minimum of detector confidence, semantic
confidence, and localization quality.

LiDAR remains an input to SLAM, costmaps, exploration, and sensor-health
checks. Raw scans are not stored or embedded in Version 1.

## 5. Structured semantics and persistent memory

### 5.1 VLM output contract

The VLM provides semantics only and links every output to a detector ID:

```json
{
  "schema_version": "environment_memory.v1",
  "scene": "hotel_lobby",
  "objects": [
    {
      "detection_id": 3,
      "label": "blue_suitcase",
      "description": "A blue suitcase beside the luggage cart.",
      "attributes": {
        "color": "blue",
        "type": "hard_shell"
      },
      "relationships": ["beside the luggage cart"],
      "useful": true,
      "confidence": 0.88
    }
  ]
}
```

Validation rules:

- Maximum eight objects, all linked to supplied detection IDs.
- Normalize labels to lowercase snake case.
- Limit descriptions to 240 characters.
- Allow at most eight string attributes and five short relationships.
- Require confidence in `[0,1]`.
- Reject coordinates, commands, arbitrary ROS names, executable content, and
  unsupported fields.
- Allow one repair retry using the schema error; discard a second invalid
  response.
- Persist only useful objects with valid geometry.

### 5.2 Storage and record representation

Use embedded Chroma with collection `environment_objects_v1`. Generate
normalized 384-dimensional cosine embeddings with
[`sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2`](https://huggingface.co/sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2).

Default storage:

```text
~/.local/share/flzat/environment_memory/<environment_id>/
├── chroma/
├── images/
├── maps/
└── manifest.json
```

Object record:

```json
{
  "schema_version": "environment_memory.v1",
  "object_id": "UUID",
  "environment_id": "hotel_demo",
  "map_id": "mapping-session UUID",
  "detector_class": "bottle",
  "label": "water_bottle",
  "description": "A blue water bottle on the counter.",
  "attributes": {"color": "blue"},
  "relationships": ["on the counter"],
  "scene": "hotel_lobby",
  "map_position": {
    "frame_id": "map",
    "x": 5.4,
    "y": 3.2,
    "z": 0.9
  },
  "robot_pose": {"x": 4.8, "y": 2.7, "z": 0.0, "yaw": 0.6},
  "first_seen_utc": "...",
  "last_seen_utc": "...",
  "first_seen_ros_ns": 0,
  "last_seen_ros_ns": 0,
  "seen_count": 1,
  "detector_confidence": 0.94,
  "semantic_confidence": 0.92,
  "localization_quality": 0.90,
  "confidence": 0.90,
  "image_ref": "images/observation_UUID.jpg",
  "embedding_model": "paraphrase-multilingual-MiniLM-L12-v2"
}
```

Embed only semantic text:

```text
<label>. <description>. Scene: <scene>.
Attributes: <attributes>. Relationships: <relationships>.
```

Coordinates, timestamps, IDs, confidence, detector data, and map identity stay
as structured metadata.

### 5.3 Basic deduplication

- Search the same `environment_id` and `map_id` for the same detector class.
- Merge only when 3D map distance is `≤0.60 m` and semantic cosine similarity
  is `≥0.80`.
- If several records qualify, choose the closest.
- Update `last_seen`, `seen_count`, confidence-weighted position, attributes,
  relationships, and embedding text.
- Replace description/image only when the new observation has higher overall
  confidence.
- Otherwise create a new UUID record.

Version 1 assumes remembered objects are static. Close identical objects and
moved objects may be associated incorrectly; robust tracking is deferred.

Write a keyframe atomically only after at least one object is accepted.
Multiple objects may reference the same observation image.

### 5.4 Build finalization and manifest

After frontier exhaustion, `memory_build_manager`:

1. Stops new triggers.
2. Waits for active/pending inference up to a configured finalization timeout.
3. Cancels work that exceeds the timeout.
4. Flushes object/image updates.
5. Saves the SLAM map.
6. Computes the saved map checksum.
7. Writes `manifest.json` atomically with:

   ```json
   {
     "schema_version": "environment_memory.manifest.v1",
     "environment_id": "hotel_demo",
     "map_id": "UUID",
     "status": "complete",
     "map_yaml": "maps/hotel.yaml",
     "map_checksum": "...",
     "database_path": "chroma",
     "object_count": 24,
     "created_at_utc": "..."
   }
   ```

An interrupted build writes `status: incomplete`. The assistant launch refuses
an incomplete manifest or a map checksum mismatch unless an explicit diagnostic
override is supplied.

During active SLAM, stored coordinates are the best `map` estimate available
at observation time. Version 1 does not retroactively correct earlier records
after pose-graph loop closure.

## 6. Retrieval and command-driven navigation

### 6.1 Retrieval

```text
user query
→ multilingual query embedding
→ Chroma cosine search
→ environment/map metadata filter
→ optional scene/time/radius post-filter
→ top five structured object records
```

`QueryMemory.srv` and a CLI client provide basic retrieval. Version 1 returns
image references but does not automatically load images into the VLM or run
multi-step agentic retrieval.

### 6.2 Assistant command flow

```text
VAD → STT transcript
    → memory_command_manager
    → structured intent: QUERY_MEMORY or NAVIGATE_TO_MEMORY
    → memory retrieval
    ├── spoken answer through TTS
    └── validated Nav2 approach goal → navigation → spoken result
```

For ambiguous matches, the command manager asks a clarification question and
does not move. The VLM may interpret language and formulate an answer, but it
never outputs a numeric pose or sends a Nav2 action directly.

For explicit navigation:

1. Retrieve the selected object's stored `map` position.
2. Generate candidate approach poses `0.8–1.2 m` from the object and oriented
   toward it.
3. Reject candidates outside the map, in occupied/inflated costmap cells, or
   without a valid path.
4. Select the nearest valid candidate from the robot's current pose.
5. Send it through `/navigate_to_pose`.
6. Report success, cancellation, or failure through TTS.

Never send the object coordinate itself as the goal because the object may
occupy that location. All motion continues through Nav2 and the existing
watchdog.

The database is read-only in assistant mode. Updating memory while servicing
commands is deferred.

## 7. Incremental implementation plan

1. **Workspace baseline and portability:** create the independent
   workspace/repository, move both planning documents into `docs/`, document
   the Windows-to-Ubuntu workflow and overlay order, pin dependencies, add
   line-ending/executable-bit policy, and freeze existing public navigation
   and companion contracts.
2. **Interfaces and reusable high-level services:** add VLM-facing interfaces,
   `speech_services.launch.py`, the environment prompt/action, and the shared
   priority-aware inference broker.
3. **Exploration:** integrate the pinned frontier explorer and build manager;
   verify readiness, repeated Nav2 goals, failure recovery, frontier
   exhaustion, and map saving.
4. **Observation acquisition:** implement RGB-D synchronization, CameraInfo and
   scan health, immutable timestamped bundles, trigger policy, bounded queues,
   and debug output.
5. **Detection and geometry:** integrate YOLOv8n behind a detector interface;
   implement depth filtering, pinhole projection, and exact timestamped TF2.
6. **Structured VLM:** implement the environment profile, schema validation,
   repair retry, background scheduling, and observation/detection ID joining.
7. **Memory manager:** implement canonical records, deduplication, keyframes,
   multilingual embeddings, Chroma persistence, manifest finalization, and
   restart recovery.
8. **Retrieval and assistant:** implement read-only retrieval, typed command
   handling, ambiguity clarification, TTS answers, safe approach-pose
   generation, and Nav2 execution.
9. **Two public launches:** complete and document both operating modes and
   their required arguments.
10. **Ubuntu end-to-end acceptance:** on the Ubuntu ROS 2 Jazzy target,
    validate memory building, restart persistence, assistant retrieval, and
    command-driven navigation in the hotel world. Record commands, results,
    artifacts, failures, and fixes in `docs/UBUNTU_TEST_LOG.md`.

## 8. Test and acceptance plan

### 8.1 Test layers

Testing is divided by what can be trusted on each platform:

1. **Windows development checks:** formatting, static analysis, interface-file
   inspection, JSON-schema fixtures, and ROS-independent Python unit tests.
2. **Ubuntu build checks:** dependency import, `rosdep`, clean `colcon build`,
   package discovery, interface generation, and launch-file syntax.
3. **Ubuntu ROS integration checks:** topics, QoS, TF tree, RGB-D
   synchronization, actions/services, lifecycle state, database persistence,
   and shared VLM scheduling.
4. **Ubuntu Gazebo acceptance:** autonomous exploration, SLAM/map saving,
   localization accuracy, deduplication, restart/retrieval, and safe Nav2
   approach behavior.

Ubuntu testing must start from a clean shell and source overlays in this exact
order:

```bash
source /opt/ros/jazzy/setup.bash
source <integrate-root>/flzat_nav_ws/install/setup.bash
source <integrate-root>/flzat-voice-ros2/install/setup.bash
source <integrate-root>/flzat_environment_memory_ws/install/setup.bash
```

The test log must identify the Git commit of all three repositories. A result
is not considered reproducible if the commits, launch arguments, world,
environment ID, or model configuration are missing.

### 8.2 Automated and integration coverage

Automated tests cover:

- Trigger thresholds, priority, cooldown, and latest-pending replacement.
- RGB-D timestamp association and invalid/stale bundle rejection.
- Depth filtering and pinhole projection with deterministic fixtures.
- Exact timestamped TF2 transformation and missing-transform rejection.
- Detector/VLM ID correlation and structured-output validation.
- VLM cancellation and voice priority in shared-server tests.
- Deduplication merge/separation and confidence-weighted updates.
- Embedding-text construction, Chroma upsert, read-only enforcement, and DB
  restart.
- Manifest atomicity, incomplete status, and map-checksum mismatch rejection.
- Query intent, navigation intent, ambiguity clarification, approach-pose
  validation, and Nav2 failure reporting.

### 8.3 Gazebo acceptance criteria

Gazebo acceptance criteria:

- Frontier exploration terminates by frontier exhaustion and saves the map.
- A test-only evaluator confirms at least 90% of reachable reference free space
  is observed. Simulator ground truth is never used by production nodes.
- At least 95% of accepted RGB-D pairs differ by no more than `80 ms`.
- Known test-object localization error is at most `0.35 m`.
- Three views of the same object produce one object ID.
- Two same-class objects separated beyond the deduplication radius remain
  separate.
- Every stored point uses frame `map`; invalid schemas and transforms never
  reach persistence.
- VLM execution never exceeds one active plus one latest pending job.
- A completed map/database survives restart and passes manifest verification.
- A known semantic query retrieves the expected object in the top three.
- Assistant mode does not modify the database.
- An explicit navigation command reaches a valid approach pose rather than the
  occupied object coordinate.
- Existing Nav2, RGB-D, watchdog, companion-pipeline, and ROS interface tests
  remain green.

### 8.4 Evidence and test-log policy

Each Ubuntu test session uses a fresh dated entry copied from the template in
`UBUNTU_TEST_LOG.md`. Preserve or link the following evidence when relevant:

- `colcon` summary and failing package logs.
- ROS node/topic/service/action lists and lifecycle state.
- `tf2` diagnostics and RGB/depth timestamp statistics.
- Gazebo world and launch arguments.
- Saved map, `manifest.json`, database object count, and checksum result.
- Example retrieval results and Nav2 final action status.
- Screenshots or rosbag references only when they materially help diagnosis.

Do not mark a test `PASS` based only on a node starting. Record the expected
behavior, observed behavior, and objective evidence. Use `BLOCKED` when the
test cannot run because of missing hardware, GPU/model files, dependencies, or
another prerequisite; do not record it as a product failure.

## 9. Recommended Version 1 MVP and deferred work

The recommended Version 1 MVP is:

```text
independent environment-memory workspace
+ two public operating-mode launches
+ frontier exploration and existing SLAM/Nav2 safety chain
+ event-triggered synchronized RGB-D snapshots
+ YOLOv8n bounding boxes
+ shared asynchronous VLM with structured semantic output
+ depth/CameraInfo projection and timestamp-correct TF2
+ object-only memory with basic spatial-semantic deduplication
+ multilingual text embeddings and embedded Chroma persistence
+ map/database manifest binding
+ read-only voice retrieval after mapping
+ validated navigation to safe approach poses
```

Defer to later versions:

- Open-vocabulary detection, class-agnostic proposals, segmentation, and
  visual embeddings.
- Continuous-video VLM inference and standalone scene/event records.
- Object tracking, re-identification, moved-object handling, and probabilistic
  association.
- Retroactive coordinate correction after SLAM loop closure.
- Stable room segmentation, topological memory, scene graphs, and graph-edge
  relationships.
- Point clouds, fused 3D object models, semantic occupancy maps, and 3D
  primitives.
- Hybrid dense/sparse retrieval, reranking, automatic image loading, temporal
  reasoning, and multi-query agents.
- STaR-style task-conditioned clustering, information-bottleneck selection,
  working memory, and agentic RAG.
- Continuous memory updates during assistant operation.
- Physical hardware calibration, runtime acceptance, privacy policy, and
  production safety certification.
