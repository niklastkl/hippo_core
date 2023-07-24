std_msgs/Header header

uint32 mask
uint32 IGNORE_POSITION = 1
uint32 IGNORE_VELOCITY = 2
uint32 IGNORE_ACCELERATION = 4

uint32 IGNORE_ATTITUDE = 8
uint32 IGNORE_RATES = 16
uint32 IGNORE_ANG_ACCELERATION = 32
uint32 IGNORE_POSE = 9
uint32 IGNORE_ACCELERATIONS = 36

uint32 IGNORE_POSITION_X = 64
uint32 IGNORE_POSITION_Y = 128
uint32 IGNORE_POSITION_Z = 256

uint32 IGNORE_ATTITUDE_X = 512
uint32 IGNORE_ATTITUDE_Y = 1024
uint32 IGNORE_ATTITUDE_Z = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

geometry_msgs/Quaternion attitude
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 angular_acceleration