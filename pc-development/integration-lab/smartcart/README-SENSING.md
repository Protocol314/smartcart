# SmartCart 통합 센싱 · Raspberry Pi 4

2026-10-27 기준 실제 Pi에서 LiDAR와 좌우 BU04를 동시에 수집하고 웹 화면을 확인했다.
현재 운영의 시작점은 [통합 README](README.md), 설치·배선·복구는
[운영 런북](<../runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.md>), 설계 설명은
[기술 백서](../docs/SMARTCART_WHITEPAPER.md)다.

## 현재 구성

| 항목 | 사용 구성 |
|---|---|
| 호스트 | Pi 4, Ubuntu 22.04.5 ARM64, ROS 2 Humble, 시스템 Python 3.10 |
| LiDAR | RPLIDAR C1 데이터 USB → `sllidar_ros2` → `/scan`, `lidar_scan`, 460800 baud |
| UWB | 좌우 BU04 독립 데이터 USB, 115200/8N1, BU03 태그는 별도 전원 |
| 처리 | 기존 C 파서 → 관측·전체 ROS 상태 → 위치 추정/군집/연관 → 읽기 전용 웹 |
| 웹 | Pi `127.0.0.1:8765`, PC는 SSH 터널 |
| ESP32 | 현재 USB 센싱의 필수 장치가 아님. 이후 제어기 통합 대상 |

BU04의 TTL 포트는 필요한 설정 조회·변경에 사용하며 평시 데이터 입력은 USB 표시 포트다.
일반 데이터 시작 스크립트는 AT 설정을 변경하지 않는다. 역할은 USB 번호가 아닌 실제 물리 라벨과
확인한 식별값으로 배정한다. 재연결 슬롯이 바뀌면 식별값을 다시 대조한다.

## 현재 Pi의 평시 시작

```bash
bash "$HOME/SmartCart-D5-v2/smartcart/scripts/start-live.sh"
```

ROS → 기존 LiDAR workspace → 통합 workspace를 적용하고 `local-config/run.json`을 사용한다.
관리 모드에서는 C1 드라이버, 좌우 수집기, 추적/웹 서버를 감독하며 Ctrl+C·기동 실패·자식 종료 때
자신이 시작한 프로세스를 정리한다. 기존 별도 드라이버를 운영하면 `lidar_mode=external`을 사용한다.
현재 Pi에는 필요한 초기 배정과 후속 업데이트가 적용돼 있다.

새 장비의 최초 설치·빌드·USB 대조·태그 설정은 런북을 따른다. 0.3.1 ZIP에는
이후 추가된 시작·배정 스크립트와 최신 화면이 없으므로 후속 파일도 반영해야 한다.
일반 배포 `smartcart-sensing-v0.1.0`과 이전 단일 ESP32 실행 절차는 현재 D5 통합 배포의 설치 경로가 아니다.

## 실측·재생 구분

| 진입점 | 범위 |
|---|---|
| `scripts/start-live.sh` | 확인한 실제 장치 구성으로 통합 시작 |
| `cart.py ... start --mode live` | 환경 source 후 같은 실행기 직접 호출 |
| 인수 없는 `scripts/start-cart.sh/.bat` | 합성 통합 재생 |
| `radar.py` | LiDAR 전용 진단 |
| `scripts/start-sensing.sh` | 보존된 단일 ESP32 진단 경로. 좌우 BU04 통합 시작용이 아님 |

런타임 npm 설치·`npm start`는 필요 없다. 시작 명령은 브라우저를 자동 실행하거나
새 PC SSH 터널을 생성하지 않는다. 기존 터널을 유지한 브라우저에서 새로고침하면 된다.

## 화면과 유효성

LiDAR는 실제 약 10Hz로 수신하며 배경 표시를 1/2Hz로 갱신한다. 현재 720 rays와
3m 보기 설정을 확인했다. 3m는 화면 반경이며 센서의 광학 측정 범위 설정이 아니다.
좌우 패널은 각 BU04의 원시 거리·각도·범위/ID 유효성·수신 경과를 최대 10Hz 표시한다.
체크섬 오류·단절·시간 초과 뒤 과거 값을 현재 측정으로 남기지 않는다.

`/api/dual-uwb`가 좌우 표시 상태, `/api/health`의 `dual_uwb`가 원시 값 없는 집계다.
기존 `uwb.state=off`는 단일 ESP32 경로의 상태로 남지만, 현재 화면은 `uwb_backend=dual`로
좌우 전용 패널을 선택한다. [표시 계약·패치](../docs/SMARTCART_DUAL_UWB_UI.md).

범위·ID가 유효한 원시 수신, 장착과 거리 가정이 검수된 위치, LiDAR와 연관된 대상 추적은
서로 다른 단계다. 동시 UWB 무효 측정 원인·정확도·실제 장착·분리/복구·장시간 전원 안정성은
후속 검수다. 모터 제어와 `/cmd_vel` 출력은 없으며 모든 구동 허가는 false다.

## 데이터와 검증

HTTP는 읽기 전용이며 루프백·Host/Origin·출력 이스케이프·요청/스트림 상한을 적용한다.
최신 거리·각도는 메모리와 현재 화면에서 처리한다. 자동 위치 기록·외부 업로드는 추가하지 않는다.
수동 PNG/JSON 내보내기와 명시한 검수 집계 저장은 사용자가 선택한 경우에만 수행한다.

실물 수신 기록과 합성·DDS·PTY 소프트웨어 검사는 [검증 기록](../docs/SMARTCART_VALIDATION.md)에
구분했다. 기존 펌웨어·GPIO·동결 보정·원본 LiDAR 폴더는 이번 통합 문서 갱신에서 변경하지 않았다.
