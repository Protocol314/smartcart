# 공식 근거와 구현 판단

확인일: 2026-10-15. 이 문서는 외부 정의와 본 프로젝트의 설계 선택을 구분합니다. 공식 문서가 접근 제한을 반환한 경우 해당 프로젝트의 원본 소스와 검색 추출 내용을 교차 확인했습니다.

| 근거 | 적용 |
|---|---|
| [ROS 2 Humble LaserScan 메시지](https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html) / [동일 정의의 공식 소스](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) | +X 기준 0 rad, +Z 기준 반시계 각도, 스캔 시간·거리 단위, 최소/최대거리 밖 값 제외 |
| [ROS 2 Humble QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | 센서 스트림의 QoS 호환성 고려 |
| [SensorDataQoS 공식 정의](https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html) | Keep Last, 깊이 5, Best Effort, Volatile을 Python 구독자 설정에 대응 |
| [ROS 2 Humble echo.py](https://github.com/ros2/ros2cli/blob/humble/ros2topic/ros2topic/verb/echo.py) | 기본 배열 생략 길이 128, `--full-length`, `--once` 지원 확인 |
| [SLAMTEC sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) | 공식 C1 지원 및 ROS 2 드라이버 경로 확인. 실제 실행 명령은 사용자의 성공한 CLI 화면을 우선했습니다. |
| 사용자 첨부 `samples/user_scan_truncated.txt` | 실제 스캔 메타데이터, Inf와 최소거리 미만 값, 배열 생략 여부 |

**자체 설계 선택:** 5 m 표시 반경, 1/2 Hz 최신 프레임 전송, 카트 중심 고정 화면, 1.5초 수신 중단 처리, SSE, 최대 8개 동시 SSE 연결, 최신 스캔의 인접점 윤곽, 1/3 m 색상 구간, 웹과 CLI 제공, 읽기 전용 API, 전륜 차동구동 도식입니다. 이 선택들이 ROS나 SLAMTEC의 필수 규격인 것은 아닙니다.

사용자 첨부 마지막 참고 이미지는 시각 방향성만 참조했습니다. 이미지의 점군을 프로그램의 실측 데이터로 넣거나, 가짜 센서 데이터로 복제하지 않았습니다. 합성 모드의 방·물체는 코드로 생성한 테스트 장면이며 화면과 내보내기에 비실측 출처가 표시됩니다.

## v1.0.1 추가 참조

- [laser_segmentation 프로젝트 원문](https://github.com/ajtudela/laser_segmentation): 인접 빔의 유클리드 거리가 임계값을 넘으면 구간을 분리하는 jump-distance 원리를 참조했습니다. 해당 패키지의 코드를 복제하거나 런타임 의존성으로 추가하지 않았습니다.
- [MDN Canvas 최적화](https://developer.mozilla.org/en-US/docs/Web/API/Canvas_API/Tutorial/Optimizing_canvas): 정적/동적 레이어 분리, 경로 배치, 불필요한 shadowBlur 생략, DPR 처리와 requestAnimationFrame을 참조했습니다.
- [MDN ResizeObserver](https://developer.mozilla.org/en-US/docs/Web/API/ResizeObserver): 컨테이너 크기에 맞춰 캔버스 버퍼를 조절합니다.

윤곽 연결식의 0.035 m·3배·2°·0.2 m 상한은 이 뷰어의 보수적인 시각화용 설계 선택입니다. C1 성능 보증 수치나 논문의 특정 알고리즘을 그대로 재현한 상수가 아닙니다. 실제 환경에서 인접한 서로 다른 물체가 붙어 보이거나 비스듬한 면이 끊길 수 있습니다.
