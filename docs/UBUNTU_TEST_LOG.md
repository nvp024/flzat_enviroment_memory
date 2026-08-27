# Version 1 Environment Memory — Ubuntu Test Log

Canonical location: `flzat_environment_memory_ws/docs/UBUNTU_TEST_LOG.md`.
Keep one dated session section for every Ubuntu validation run. Do not
overwrite previous results.

Allowed status values: `NOT_RUN`, `PASS`, `FAIL`, `BLOCKED`.

## Current Phase 1–5 and Phase 7 Ubuntu validation

The current source includes Phase 7 persistence but still requires a Phase 6
localized-semantic producer for live ingestion. Before using the later
full-system sections in this log, validate the available integration with:

```bash
source /opt/ros/jazzy/setup.bash
source <integrate-root>/flzat_nav_ws/install/setup.bash

cd <integrate-root>/flzat-voice-ros2
colcon build --symlink-install
source <integrate-root>/flzat-voice-ros2/install/setup.bash

cd <integrate-root>/flzat_environment_memory_ws
vcs import < environment_memory.repos
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-detector.txt
python -m pip install -r requirements-memory.txt
python tools/fetch_yolov8n.py
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 interface show robot_interfaces/action/AnalyzeEnvironment
ros2 interface show environment_memory_interfaces/msg/ExplorationStatus
ros2 interface show environment_memory_interfaces/msg/GeometricObjectObservation
ros2 interface show environment_memory_interfaces/msg/LocalizedObjectObservation
ros2 launch environment_memory exploration_observation.launch.py \
  environment_id:=hotel_demo map_id:=<mapping-session-uuid> \
  headless:=true use_rviz:=false
```

For the Phase 5 run, record `/exploration/status`,
`/environment_memory/observation_status`,
`/environment_memory/debug_image`, the required sensor topics, the complete TF
chain, `/environment_memory/geometric_observations`, Nav2 lifecycle states,
frontier completion, and the saved map result. Confirm that each accepted
geometry message uses the RGB stamp for TF-derived outputs and frame `map` for
`map_position` and `robot_pose`.

For Phase 7, also record `/environment_memory/status`, the Chroma collection
count, keyframe paths, incomplete-to-complete manifest transition, map YAML
checksum, a same-object merge, a separated same-class object, and recovery from
an interrupted build using the same `environment_id` and `map_id`.
The `autonomous_memory_build.launch.py` and `memory_assistant.launch.py`
commands in the full Version 1 template remain `NOT_RUN` until their later
implementation phases are complete.

## Test session template

Copy this entire section for a new test session.

### Session: YYYY-MM-DD — short purpose

#### 1. Environment

```text
Tester:
Date/time and timezone:
Ubuntu version:
Kernel:
ROS distribution: Jazzy
RMW implementation:
Gazebo version:
Python version:
GPU and driver:
CUDA version, if used:
VLM backend/model:
Detector weights/version:
Embedding model/revision:
Test world:
Environment ID:
Map ID, if reusing a database:
```

Repository revisions:

```text
flzat_nav_ws branch/commit:
flzat-voice-ros2 branch/commit:
flzat_environment_memory_ws branch/commit:
Dirty files, if any:
```

#### 2. Setup and dependency check

Commands run:

```bash
source /opt/ros/jazzy/setup.bash

cd <integrate-root>/flzat_nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

cd <integrate-root>/flzat-voice-ros2
source <integrate-root>/flzat_nav_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

cd <integrate-root>/flzat_environment_memory_ws
source <integrate-root>/flzat-voice-ros2/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

```text
Status: NOT_RUN
Expected: all dependencies resolve and all three overlays build successfully.
Observed:
Evidence/log path:
Failure or blocker:
Next action:
```

#### 3. Automated tests

Commands run:

```bash
cd <integrate-root>/flzat_nav_ws
colcon test
colcon test-result --verbose

cd <integrate-root>/flzat-voice-ros2
colcon test
colcon test-result --verbose

cd <integrate-root>/flzat_environment_memory_ws
colcon test
colcon test-result --verbose
```

```text
Status: NOT_RUN
Expected: no failed tests in any overlay.
Observed:
Passed/failed/skipped totals:
Failing test names:
Evidence/log path:
Failure or blocker:
Next action:
```

#### 4. Preflight ROS contracts

Verify before autonomous motion:

- RGB: `/camera/color/image_raw`
- Depth: `/camera/depth/image_raw`
- Camera info: `/camera/camera_info`
- LiDAR: `/scan`
- Odometry: `/odom`
- Map: `/map`
- TF chain: `map -> odom -> base_footprint -> base_link -> camera_optical_frame`
- Nav2 action: `/navigate_to_pose`
- Structured VLM action: `/vlm/analyze_environment`
- Phase 5 geometry: `/environment_memory/geometric_observations`
- Frozen semantic input: `/environment_memory/vlm_observations`
- Phase 6 output: `/environment_memory/localized_observations`
- Semantic drain status: `/environment_memory/semantic_status`
- Safe velocity path ends at `/cmd_vel_safe`

```text
Status: NOT_RUN
Expected: required topics and TF are live, frame IDs match, Nav2 is active,
and velocity commands pass through the watchdog contract.
Observed:
Commands/evidence:
Failure or blocker:
Next action:
```

#### 5. Autonomous memory-build launch

Launch command and all non-default arguments:

```bash
ros2 launch environment_memory autonomous_memory_build.launch.py \
  environment_id:=hotel_demo map_id:=<mapping-session-uuid>
```

```text
Status: NOT_RUN
Expected: the robot explores through Nav2, avoids obstacles, accepts bounded
RGB-D observations, runs no voice pipeline, and finalizes the map and DB after
frontier exhaustion.
Observed:
Start/end time:
Exploration goal counts:
Accepted/rejected observation counts:
Maximum active/pending VLM jobs:
Schema repair attempts/rejections:
Localized semantic publication count:
Saved artifact directory:
Failure or blocker:
Next action:
```

#### 6. RGB-D, TF, and object-localization evidence

```text
Status: NOT_RUN
Expected: at least 95% of accepted RGB-D pairs are within 80 ms; persisted
objects use exact timestamped TF in frame `map`; known-object error is at most
0.35 m.
Observed RGB-D percentile/count:
Observed TF failures/rejections:
Test object and reference position:
Estimated position:
Position error:
Evidence/log path:
Failure or blocker:
Next action:
```

#### 7. Deduplication and persistence

```text
Status: NOT_RUN
Expected: three views of one object produce one ID; sufficiently separated
same-class objects remain separate; completed data survives process restart.
Observed object IDs and seen counts:
Chroma object count before restart:
Chroma object count after restart:
Manifest status:
Map checksum result:
Evidence/log path:
Failure or blocker:
Next action:
```

#### 8. Memory-assistant launch and retrieval

Phase 9 public assistant preflight:

```bash
ros2 launch environment_memory memory_assistant.launch.py \
  environment_id:=hotel_demo map_id:=<mapping-session-uuid>
ros2 service type /environment_memory/query
ros2 run environment_memory query_memory "blue water bottle" \
  --environment-id hotel_demo --map-id <mapping-session-uuid>
```

Record the manifest and Chroma directory hashes before and after queries to
confirm that assistant mode did not modify the completed memory artifacts.

Launch command and all non-default arguments:

```bash
ros2 launch environment_memory memory_assistant.launch.py \
  environment_id:=hotel_demo
```

```text
Status: NOT_RUN
Expected: saved map and matching read-only DB load; a known query retrieves the
expected object in the top three; assistant mode makes no database changes.
Query/transcript:
Top results and scores:
Spoken/text response:
DB checksum/count before and after:
Evidence/log path:
Failure or blocker:
Next action:
```

#### 9. Command-driven navigation

```text
Status: NOT_RUN
Expected: an explicit navigation command creates a collision-free approach
pose 0.8–1.2 m from the object, uses Nav2, and reports the final result. An
ambiguous request asks for clarification and causes no motion.
Command/transcript:
Retrieved object ID and position:
Selected approach pose:
Path validation result:
Nav2 final status:
Observed final distance/orientation:
Ambiguity/no-motion check:
Evidence/log path:
Failure or blocker:
Next action:
```

#### 10. Session summary

```text
Overall status: NOT_RUN
Passed checks:
Failed checks:
Blocked checks:
New defects:
Code/config changes needed:
Retest scope:
Artifact backup location:
Additional notes:
```

## Test history

Add one summary row after each session.

| Date | Commits tested | Build | Automated | Memory build | Assistant | Navigation | Overall |
|---|---|---:|---:|---:|---:|---:|---:|
| YYYY-MM-DD | nav / voice / memory | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN |
