# One backend detection, in the frame of the parent ObservationArray header.
string class_id                       # model label, e.g. red_buoy
float32 score                         # 0..1
string backend_id                     # backend's own object id; "" = backend has none
geometry_msgs/Point position          # box centre, m, header frame
float64[9] position_covariance        # row-major 3x3 m^2; all zero = backend gives none
geometry_msgs/Vector3 size            # m, header-frame axes; zeros = unknown
uint8 VELOCITY_NONE=0
uint8 VELOCITY_RELATIVE=1             # relative to the camera, header-frame axes
uint8 VELOCITY_WORLD=2                # ground-fixed, header-frame axes
uint8 velocity_kind
geometry_msgs/Vector3 velocity        # m/s
float64[9] velocity_covariance        # all zero = use tracker YAML velocity_sigma
bool has_box2d
float32[4] box2d                      # u0 v0 u1 v1, normalized 0..1 of the source image
