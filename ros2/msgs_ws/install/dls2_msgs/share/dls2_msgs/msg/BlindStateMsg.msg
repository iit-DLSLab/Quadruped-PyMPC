# Header
string frame_id
uint32 sequence_id
float64 timestamp

string robot_name

string[12] joints_name
float64[12] joints_position
float64[12] joints_velocity
float64[12] joints_acceleration
float64[12] joints_effort
float64[12] joints_temperature

float64[4] feet_contact
float64[12] current_feet_positions