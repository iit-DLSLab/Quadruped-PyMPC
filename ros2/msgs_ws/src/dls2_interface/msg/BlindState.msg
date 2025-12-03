# Header
string frame_id
uint32 sequence_id
float64 timestamp

string robot_name

string[] joints_name
float64[] joints_position
float64[] joints_velocity
float64[] joints_acceleration
float64[] joints_effort
float64[] joints_temperature

bool[] feet_contact
float64[] current_feet_positions