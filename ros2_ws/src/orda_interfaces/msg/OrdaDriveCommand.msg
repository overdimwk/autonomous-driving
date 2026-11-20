# stamp: latest received sensor acquisition time (Unity monotonic seconds).
# frame_id must be base_link. sequence must increase for this simulator session.
std_msgs/Header header
uint64 sequence
float32 target_speed_mps
# Dimensionless: -100 full left, +100 full right. Not rad/s or degrees.
float32 steering_command
