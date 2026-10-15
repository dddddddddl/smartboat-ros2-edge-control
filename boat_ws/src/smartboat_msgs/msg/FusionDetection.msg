std_msgs/Header header

uint8 GO_STRAIGHT=0
uint8 TURN_LEFT=1
uint8 TURN_RIGHT=2

bool sensor_ready
bool obstacle_detected
float32 distance_m
uint8 decision
uint32 obstacle_pixels
string reason
float32 inference_ms
