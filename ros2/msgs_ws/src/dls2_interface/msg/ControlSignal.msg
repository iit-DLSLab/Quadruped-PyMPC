# Header
string frame_id
uint32 sequence_id
float64 timestamp

uint8 control_mode;

float64[] joints_position;
float64[] joints_velocity;
float64[] joints_torques;
float64[] kp;
float64[] kd;