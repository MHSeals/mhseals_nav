std_msgs/Header header                # stamp = image capture time; frame_id = real camera frame in TF
string backend                        # zed_native | sim  (= perception source name in health YAML)
float32 processing_latency            # s, capture -> this publish, measured by the backend
Observation[] observations            # an EMPTY array is published for every frame with no hits
