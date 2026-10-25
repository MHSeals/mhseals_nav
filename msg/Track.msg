uint32 id                             # tracker id, never reused within a run
string class_id
float32 class_score                   # EMA of associated scores
string backend_id                     # last associated backend id ("" if none)
uint8 MOTION_UNKNOWN=0
uint8 MOTION_STATIC=1
uint8 MOTION_MOVING=2
uint8 motion
uint8 SOURCE_LIVE=0
uint8 SOURCE_PRIOR=1                  # loaded from the course map, not re-observed this run
uint8 source
geometry_msgs/PoseWithCovariance pose     # map frame; orientation identity; xy block filled, z from obs
geometry_msgs/TwistWithCovariance twist   # map frame; linear x,y and their 2x2 block
geometry_msgs/Vector3 size
float32 confidence                    # existence 0..1 after decay
float32 speed_bound                   # m/s the speed must exceed to count as moving evidence
builtin_interfaces/Time first_seen    # age = header.stamp - first_seen
builtin_interfaces/Time last_seen
uint32 hits
