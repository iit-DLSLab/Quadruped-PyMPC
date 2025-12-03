# Header
string frame_id
uint32 sequence_id
float64 timestamp

string robot_name

# Base pose
Pose pose

# Base velocity
Screw velocity

# Base acceleration
Screw acceleration

# Stance status
bool[] stance_status
