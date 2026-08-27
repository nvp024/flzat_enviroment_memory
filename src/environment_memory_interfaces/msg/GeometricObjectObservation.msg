# Internal Phase 5 output. Phase 6 joins semantics by both IDs before creating
# LocalizedObjectObservation records.
string observation_id
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time depth_stamp
robot_interfaces/ObjectDetection2D detection
geometry_msgs/PointStamped camera_position
geometry_msgs/PointStamped map_position
geometry_msgs/PoseStamped robot_pose
float32 valid_depth_ratio
float32 depth_mad_m
float32 localization_quality
float32 geometric_confidence
