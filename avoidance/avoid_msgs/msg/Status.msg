# Published by avoid_nav at ~10 Hz. The web UI shows this, not its own guess of the state.
Header header

uint8 IDLE=0
uint8 MAPPING=1      # lidar backup only: standing still at the start, building the anchored map
uint8 RUNNING=2
uint8 FINISHED=3     # the footprint touched the goal line; braking to a stop
uint8 SAFE_STOP=4    # stopped because something is wrong; see reason
uint8 state
string state_name
string reason        # why SAFE_STOP, or the result of the last command
string sensor_config # "camera" | "lidar"

# Age of the newest message from each input [s]; negative = never received.
float32 age_obstacles
float32 age_boundary
float32 age_odom
float32 age_imu

geometry_msgs/Pose2D pose   # localisation estimate in the field frame
float32 pose_std_x
float32 pose_std_y
float32 pose_std_theta

geometry_msgs/Twist cmd     # last velocity command sent (base frame, REP 103)
float32 elapsed             # seconds since RUNNING started (0 when not running)
