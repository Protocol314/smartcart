# 스마트카트 최신 코드 목록

2026-11-26 기준 · GitHub `Protocol314/smartcart` `main` (`af6b1b2`)

## PC 개발용

| 항목 | 역할 | 코드 |
| --- | --- | --- |
| 센싱·추적 | ROS 2 센싱, 단일·좌우 BU04 수집, LiDAR 군집과 UWB 대상 추적 | [`sensing.py`](../pc-development/SmartCart-D5-v2/smartcart/sensing.py) · [`smartcart_uwb/`](../pc-development/SmartCart-D5-v2/smartcart/smartcart_uwb/) · [`smartcart_tracking/`](../pc-development/SmartCart-D5-v2/smartcart/smartcart_tracking/) |
| 센서 드라이버 | C1 LiDAR `/scan` 발행, BU04 PDOA 프레임 파싱 | [`sllidar_c1_launch.py`](../pc-development/smartcart_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py) · [`bu04_pdoa.c`](../pc-development/SmartCart-D5-v2/components/c_key_core/bu04_pdoa.c) |
| Pi 기반 가상 조종 | MOCK 출력, LiDAR·BU04 이상 잠금, 2초 PWM 응답 감시 | [`manual.sh`](../pc-development/manual.sh) · [`sensor_guard.py`](../pc-development/SmartCart_ManualDrive_180/smartcart/smartcart_motor/sensor_guard.py) · [`manual_controls.js`](../pc-development/SmartCart_ManualDrive_180/smartcart/web/js/manual_controls.js) |
| 18084 가상 화면 | 실측 센서 표시, 수동 조종·자율주행 미리보기, 연결·정지 진단. Pi 적용 미확인 | [`virtual_lan_server.py`](../pc-development/virtual-lab-18084/virtual_lan_server.py) · [`web/`](../pc-development/virtual-lab-18084/web/) · [`tests/`](../pc-development/virtual-lab-18084/tests/) |
| 통합 개발 기준 | 센싱·추적·수동/추종 감독기, 안전 상태 변환, 가상 출력 | [`integration-lab/smartcart/`](../pc-development/integration-lab/smartcart/) · [`supervisor.py`](../pc-development/integration-lab/smartcart/smartcart_motor/supervisor.py) · [`safety_provider.py`](../pc-development/integration-lab/smartcart/smartcart_motor/safety_provider.py) |
| 회귀 시험 | Python·JavaScript, BU03·BU04 C 호스트 회귀 | [`smartcart/tests/`](../pc-development/integration-lab/smartcart/tests/) · [`c_key_core/`](../pc-development/integration-lab/components/c_key_core/) · [`Windows`](../pc-development/integration-lab/tests/run_host_tests.bat) · [`Linux`](../pc-development/integration-lab/tests/run_host_tests.sh) |
| 이전 추종 코드 | PR #1의 Python 소스 3개 보존 | [`smartcart_latest_pr/`](../pc-development/contributions/smartcart_latest_pr/) |

## Pi 직접 실행 경로

Raspberry Pi 4 · Ubuntu 22.04 ARM64 · ROS 2 Humble · C1 LiDAR · 단일 BU04 · BU03 태그

| 항목 | Pi 경로 | 확인 상태 |
| --- | --- | --- |
| 센싱 시작 | `~/smartcart/sensors.sh` → `SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh` | 8765 HTTP 응답 보고. 현행 스크립트 일치 및 측정 유효성 미확인 |
| 실제 조종 서버 | `~/smartcart/real.sh` → `~/smartcart/releases/real-uwb-v1/server.py` | 11월 24일 8081 HTTPS 접속, GPIO 프로세스 실행 및 UWB 추종 시험 보고. Pi 최종 소스·설정 대조와 실차 안전 검수 미완료 |
| 현장 설정 | `~/smartcart/releases/real-uwb-v1/follow_config.json` · `~/smartcart/releases/real-uwb-v1/live_zone_modules/cart_geometry.json` | Pi에 존재 확인. 장착값·설정 유효성 검수 미완료 |

실차 확인 항목은 센서 무효·ID 불일치·시간 초과 시 정지와 재활성화, 2초 PWM 응답 감시, 모터 극성, 물리 비상정지다. [현장 진행 기록](../SMARTCART_PROGRESS.md) · [Pi 운영 런북](<../runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.md>)

## GitHub의 Pi 소스

| 항목 | 역할·상태 | 코드 |
| --- | --- | --- |
| 11월 24일 현장 스냅샷 | Pi에서 추출한 소스 기반. 일부 파일이 현행 Pi와 다르며 아래 추종 서버 검토본은 GPIO 잠금·MOCK 전용 | [`SNAPSHOT-REVIEW.md`](../pi-runtime/pi-field-snapshot/SNAPSHOT-REVIEW.md) |
| 센싱 | 단일 BU04 실행기·ROS 센싱·C1 LiDAR 시작 | [`sensors.sh`](../pi-runtime/pi-field-snapshot/sensors.sh) · [`start-single-left-trial.sh`](../pi-runtime/pi-field-snapshot/SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh) · [`sensing.launch.py`](../pi-runtime/pi-field-snapshot/SmartCart-D5-v2/smartcart/launch/sensing.launch.py) · [`sllidar_c1_launch.py`](../pi-runtime/pi-field-snapshot/smartcart_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py) |
| 8081 서버·제어 | HTTPS 조종·추종 서버, 제어 브리지, UWB 유효성·만료·이상치 처리 | [`real.sh`](../pi-runtime/pi-field-snapshot/real.sh) · [`server.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/server.py) · [`runtime.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/runtime.py) · [`uwb_control.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/uwb_control.py) |
| LiDAR 판정 | 장애물·미관측 구간, 정지·회전 영역 및 경로 계산 | [`lidar_guard.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/lidar_guard.py) · [`reactive_lidar.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/reactive_lidar.py) · [`zone_live.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/zone_live.py) · [`segment_navigation.py`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/segment_navigation.py) |
| 화면·검증 | 조종 화면, UWB 추종 UI, 합성 입력·MOCK 회귀 | [`web/index.html`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/web/index.html) · [`uwb_real.js`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/web/js/uwb_real.js) · [`tests/`](../pi-runtime/pi-field-snapshot/releases/real-uwb-v1/tests/) |
| Pi 가상 실행 후보 | ROS·BU04 빌드 준비, 읽기 전용 센싱 진단, 8081 MOCK 조종 | [`prepare_ros.sh`](../pi-runtime/prepare_ros.sh) · [`check_pi.sh`](../pi-runtime/check_pi.sh) · [`manual.sh`](../pi-runtime/manual.sh) |
| 11월 19일 내보내기 | 기존 Pi 소스 보존본. 현행 설치본과 해시 대조 전 | [`pi-export-reference/`](../pi-runtime/pi-export-reference/) |
