# Internal Phase 6 handoff: one frozen annotated image and all detections that
# already have valid Phase 5 map-frame geometry.
string observation_id
builtin_interfaces/Time observation_stamp
sensor_msgs/CompressedImage image
robot_interfaces/ObjectDetection2D[] detections
