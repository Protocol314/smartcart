# 장치의 측정 시각이 아닌 호스트 수신 시각을 보존하는 읽기 전용 계약이다.
string schema
string sensor_id
string frame_id
string adapter_session
string source_mode
string timestamp_source
uint64 rx_monotonic_ns
bool source_rx_available
uint64 source_rx_monotonic_ns
uint64 valid_until_monotonic_ns
bool device_timestamp_available
uint64 device_timestamp_ns
uint64 sample_sequence
uint64 stream_epoch
# 불량 프레임의 관측은 원시 필드가 없을 수 있다.
bool frame_available
uint16 tag_address
int16 logical_target_id
uint8 device_sequence
# 센서가 보고한 원시 거리이며 수평 거리 보정과 장착 변환을 적용하지 않는다.
float64 range_m
# ROS의 좌측 양수로 부호만 바꾼 센서 로컬 방위각이다. 장착 yaw는 아직 미적용이다.
float64 bearing_rad
int32 raw_angle_deg
int32 first_path_power
int32 rx_level
bool measurement_valid
bool id_valid
bool valid
string invalid_reason
bool mounting_verified
string calibration_profile
# 이 패키지는 구동 허가를 생성하지 않으며 항상 false이다.
bool drive_authorized
