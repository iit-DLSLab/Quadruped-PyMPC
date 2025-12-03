# Header
string frame_id
uint32 sequence_id
float64 timestamp

Pose com_pose

Screw com_vel

Screw com_acc


float64[] joints_position
float64[] joints_velocity
float64[] joints_acceleration
float64[] joints_effort
float64[] kp
float64[] kd

float64[6] wrench

bool[] stance_legs

float64[] nominal_touch_down
float64[] touch_down
float64[] swing_period

float64[] normal_force_max
float64[] normal_force_min

