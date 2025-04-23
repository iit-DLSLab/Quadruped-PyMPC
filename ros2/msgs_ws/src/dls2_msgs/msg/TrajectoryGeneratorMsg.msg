# Header
string frame_id
uint32 sequence_id
float64 timestamp

float64[3] com_position
float64[4] com_orientation

float64[3] com_linear_velocity
float64[3] com_angular_velocity

float64[3] com_linear_acceleration
float64[3] com_angular_acceleration

float64[12] joints_position
float64[12] joints_velocity
float64[12] joints_acceleration
float64[12] joints_effort

float64[6] wrench

bool[4] stance_legs

float64[12] nominal_touch_down
float64[12] touch_down
float64[4] swing_period

float64[4] normal_force_max
float64[4] normal_force_min

float64[12] kp
float64[12] kd