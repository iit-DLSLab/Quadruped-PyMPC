# Header
string frame_id
uint32 sequence_id
float64 timestamp

string robot_name

float64[3] position
float64[4] orientation

# Base velocity
float64[3] linear_velocity
float64[3] angular_velocity

# Base acceleration
float64[3] linear_acceleration
float64[3] angular_acceleration

# Stance status
float64[4] stance_status
