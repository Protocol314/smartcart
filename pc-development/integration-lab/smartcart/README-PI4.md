# Smart Cart D5-v2 · Raspberry Pi 4 센싱 배포

배포 묶음 0.3.1의 대상은 Raspberry Pi 4, Ubuntu 22.04 ARM64(aarch64), ROS 2 Humble,
시스템 Python 3이다. 기존 런타임 패키지 버전은 0.1.0을 유지한다.
좌우 BU04와 중앙 LiDAR의 읽기 전용 수집·추적·검수를 제공하며 모터 출력은 없다.
ESP32 펌웨어, BU03 설정, 기존 동결 보정값을 설치하거나 수정하지 않는다.

실제 ARM64 ELF를 교차 빌드하고 QEMU에서 C 회귀와 파서 ABI를 실행했다.
ROS·PTY 검사는 Ubuntu/Humble x86_64에서 실행했다. 2026-10-27에는 실제 Pi 설치·빌드·USB 역할 배정과
LiDAR·좌우 BU04 동시 수신 및 웹 표시를 확인했다. 실제 정확도·장착·장시간 안정성·주행 검수는 남아 있다.
QEMU·합성·DDS 검사 결과와 이 실물 수신 기록은 [검증 기록](../docs/SMARTCART_VALIDATION.md)에 구분한다.

## 이미 설치된 현재 Pi의 평시 시작

```bash
bash "$HOME/SmartCart-D5-v2/smartcart/scripts/start-live.sh"
```

설치·빌드·장치 배정과 후속 패치를 마친 현재 Pi에서 유효하다. ROS 환경 source를 포함한다.
관리 모드에서 C1 드라이버·좌우 수집기·추적/웹 서버가 시작되며 종료는 같은 창의 Ctrl+C다.
PC 화면은 기존 SSH 터널을 유지하고 `http://127.0.0.1:8765/`에서 확인한다.

0.3.1 기반 ZIP은 보존된 파일이다. **새 설치는 USB 링크 열거 수정, 시작·배정 도구와
좌우 화면·재시작 패치를 추가 반영해야 현재 Pi 구성이 된다.** 정확한 순서는
[운영 런북 6.3절](<../runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.md#63-031-설치-후-후속-파일-반영>)을 따른다.
이미 운영 중인 Pi에는 아래 최초 설치·초기화·장치 배정을 반복하지 않는다.

## Pi·센서 연결 전 준비 (0.3.1)

Pi가 없어도 개발 PC에서 합성 통합 GUI·실행 계획·고장/복구·HTTP 종료를 확인할 수 있다.
통합 실행은 `cart.py`, 사전 검증은 `scripts/verify-offline.py`를 사용한다.
기본 실행은 재생이며 실제 수집·드라이버·GUI 시작은 `--mode live`를 명시한다.
Windows 개발 저장소는 `smartcart/scripts/start-cart.bat`, Linux 배포본은
`smartcart/scripts/start-cart.sh`를 인수 없이 실행해 재생을 시작한다.
독립 절차는 배포 루트의 `smartcart/README-OFFLINE.md`를 따른다.
개발 저장소에서는 같은 안내가 이 파일 옆 `README-OFFLINE.md`에 있다.
기존 0.2.0 배포 파일은 별도 보존하며 덮어쓰지 않는다.

## 배포 파일 확인과 설치

같은 이름의 ZIP, `-install.py`, `.zip.sha256` 세 파일을 Pi 일반 계정 홈에 복사한다.
해시 파일의 첫 번째 값과 전달받은 ZIP 해시를 대조한다. 해시는 서명이나 발신자 인증이 아니다.
아래 값은 보존된 0.3.1 ZIP의 고정 해시다. 다른 배포 파일에는 그 파일의 신뢰할 수 있는 해시를 사용한다.

```bash
SC_ZIP_SHA256='ee1aff53c792ae0a3beb2a90e632e6e42a5f386358e8afeb4ede5a3a599b9ec5'
python3 smartcart-d5-v2-pi4-arm64-v0.3.1-install.py --archive smartcart-d5-v2-pi4-arm64-v0.3.1.zip --sha256 "$SC_ZIP_SHA256" --verify-only
python3 smartcart-d5-v2-pi4-arm64-v0.3.1-install.py --archive smartcart-d5-v2-pi4-arm64-v0.3.1.zip --sha256 "$SC_ZIP_SHA256"
```

`--verify-only`는 다른 OS에서도 설치 없이 ZIP·경로·용량·파일 해시·ARM64 ELF를 검사한다.
설치는 Ubuntu 22.04 aarch64 일반 계정에서만 허용한다. `sudo`로 실행하지 않는다.
새 `~/SmartCart-D5-v2`만 만들며 기존 폴더를 덮어쓰지 않는다.
자동 apt 설치, 서비스 등록, udev 변경, 드라이버 실행, USB 개방은 하지 않는다.
일반 사용자에게 pyserial과 기존 ROS Humble·colcon·ROS 인터페이스 생성 도구가 준비되어 있어야 한다.

```bash
cd ~/SmartCart-D5-v2
bash smartcart/scripts/build-pi4.sh
source workspace/install/setup.bash
cd smartcart
python3 field_audit.py doctor --require-pi4
```

빌드 스크립트는 대상 환경·파서 소스·ARM64 라이브러리 해시를 검사하고 새 `workspace`에
`smartcart_uwb_interfaces`, `smartcart_sensing`을 빌드한다. 장치를 시작하지 않는다.
실패한 빌드도 폴더를 자동 삭제하지 않는다. 로그를 확인한 뒤 사용자가 새 경로를 준비한다.
소스 실행과 설치본의 `ros2 run smartcart_sensing smartcart-field`는 같은 검수 코어를 사용한다.

## 장치 배정과 장착 실측

통합 실행기의 최초 설정은 런북 9절의 `cart.py init`과 물리 역할 대조 절차를 권장한다.
아래 수동 파일 복사는 개별 현장 검수 도구의 설정 예시이며 통합 실행에는 `run.json`도 필요하다.
기존 `local-config`가 있으면 생성·덮어쓰기를 반복하지 않는다.

```bash
python3 field_audit.py inventory --show-identifiers
mkdir -m 700 local-config
cp config/dual_uwb.d5.example.json local-config/mounting.json
cp config/field_acceptance.example.json local-config/field.json
cp config/uwb_estimation.example.json local-config/uwb.json
cp config/target_tracking.example.json local-config/tracking.json
```

`inventory`는 pyserial 포트 목록만 읽으며 포트를 열지 않는다. 기본 출력은 식별값의 해시다.
`--show-identifiers`를 명시했을 때만 포트·원문 식별자를 표준오류 화면에 표시한다.
각 USB를 하나씩 대조하여 좌·우·중앙 역할을 확정한다. 포트 열거 순서로 자동 배정하지 않는다.
가능하면 운영체제가 제공한 안정적인 `/dev/serial/by-id/...` 경로를 사용한다.

좌우는 `mounting.json`의 각 `sensors` 항목, 중앙은 `field.json`의 `lidar_device`에
확인한 `port`, 정확한 `expected_device_identity`를 넣고 실제 대조한 경우만
`device_identity_verified=true`로 기록한다. 같은 USB의 별칭·복수 인터페이스를 여러 센서로
배정하면 거부한다. 이 검사는 USB 열거 식별이며 BU04 내부 ID를 AT로 조회하지 않는다.
현재 태그의 허용 주소도 기존 수집 설정에서 확인하며 자동 학습·변경하지 않는다.

좌표는 후륜축 실제 중앙 `base_link`, +X 전방·+Y 좌측·+Z 위쪽, m·deg다.

| 센서 | 설계상 X/Y/Z(m) | yaw(deg) | 실측할 기준점 |
|---|---|---|---|
| uwb_left | 0.756 / 0.244 / 0.286 | 0 | RF 위상 중심 |
| uwb_right | 0.756 / -0.244 / 0.286 | 0 | RF 위상 중심 |
| lidar_center | 0.817 / 0 / 0.2678 | 0 | 스캔 원점 |

이 값은 CAD 제안이다. UWB의 기존 CAD PCB 중심과 RF 위상 중심은 다를 수 있다.
`mounting.json`에 실측 좌표·yaw와 해당 `mounting_verified`를 기록한다.
`field.json`에는 원점·축 확인, 세 역할의 translation/yaw/height/level 확인,
`reference_point`(UWB `rf_phase_center`, LiDAR `scan_origin`), 짧은 `evidence_note`를 기록한다.
증빙 메모에 사람 이름·태그 주소·이동 경로를 넣지 않는다.
기본 기준은 좌우 간격 최소 0.05m, 중앙 LiDAR의 좌우 편차 최대 0.05m다.

`uwb.json`의 거리 모델은 기본 `unknown`, 가정 검증은 false다.
실측으로 `range_model="horizontal"`(수평 거리) 또는 `"slant_known_height"`(높이를 아는 공간 거리)를
확인한 후에만 설정한다. 후자는 `tag_height_m`에 지면 높이가 아닌 **base_link 기준 태그의 양수 Z**를
지정하고 `tag_height_verified=true`도 필요하다. horizontal에서는 태그 높이를 추정하지 않는다.
대응하는 `field.json`의 `range_model_verified`와 추정 설정의 `assumptions_verified`도
근거가 있을 때만 true로 바꾼다. 설정 변경은 로컬 JSON에만 적용한다.
기존 동결 각도 보정값을 가져오거나 하드웨어 보정값을 쓰지 않는다.

```bash
python3 field_audit.py check --mounting-config local-config/mounting.json --field-config local-config/field.json --uwb-config local-config/uwb.json --tracking-config local-config/tracking.json
```

장치 누락·중복, 좌우 반전, 좌표/프레임 불일치, 미확인 증빙은 구분하여 표시한다.
수치가 유효해도 실측 증빙이 없으면 준비 완료가 아니다. 설정 확인만으로 실물 검수를 완료하지 않는다.

## 읽기 전용 실물 관측

현재 소스의 실측 전용 시작 스크립트를 설치했다면 환경 source와 통합 시작은 다음 한 명령으로 수행한다. 0.3.1 ZIP에는 이 추가 파일이 없으므로 별도 복사가 필요하다. USB 배정과 로컬 설정을 먼저 완료한다.

```bash
bash "$HOME/SmartCart-D5-v2/smartcart/scripts/start-live.sh"
```

이 명령은 관리 모드에서 LiDAR 드라이버·좌우 UWB·추적 GUI를 함께 시작한다. 기존 단독 드라이버·수집기는 먼저 정상 종료해야 한다. 아래 개별 명령은 원인 진단이나 외부 드라이버 운영에 사용한다.

모든 프로세스는 같은 Pi의 같은 ROS 도메인을 사용하고 각 터미널에서 환경을 source한다.
`export ROS_LOCALHOST_ONLY=1`을 적용한다. 한 USB는 기존 소유 프로세스 하나만 연다.
기존 배선·전원과 현재 구동 허가 false를 유지한다.

터미널 1: 확인한 좌우 설정으로 기존 읽기 전용 수집기를 명시적으로 실행한다.

```bash
ros2 run smartcart_sensing smartcart-uwb-ros --config "$HOME/SmartCart-D5-v2/smartcart/local-config/mounting.json"
```

터미널 2: 이미 검수한 기존 LiDAR 드라이버를 실행한다. 이 묶음에는 드라이버를 포함하지 않는다.
기존 `sllidar_ros2` 설치·포트 설정을 사용하며 중복 실행하지 않는다.
드라이버의 `LaserScan.header.frame_id`, `mounting.json`의 `lidar.frame_id`,
`field.json`의 `expected_lidar_frame_id`를 정확히 맞춘다(예제 `lidar_scan`). 자동 TF 변환은 없다.

터미널 3: 기존 추적·GUI 연결이 필요하면 `smartcart` 디렉터리에서 실행한다.

```bash
python3 sensing.py serve --mode ros --uwb-mode off --tracking-mode ros --tracking-mounting-config local-config/mounting.json --tracking-uwb-config local-config/uwb.json --tracking-config local-config/tracking.json
```

GUI는 같은 Pi의 `http://127.0.0.1:8765`다. 후속 업데이트에서 좌우 원시 패널은 현재 거리·각도·
범위/ID 유효성과 경과를 표시한다. 기본 미검증 거리 모델에서는 대상 위치가 나오지 않는 것이 정상이다.
검수 도구 자체는 위치를 추정하지 않으며 GUI를 필요로 하지 않는다.

터미널 4: `smartcart` 디렉터리에서 새 보고서 이름을 지정한다.

```bash
python3 field_audit.py monitor --mounting-config local-config/mounting.json --field-config local-config/field.json --uwb-config local-config/uwb.json --tracking-config local-config/tracking.json --seconds 300 --interactive-markers --progress --output field-report-01.json
```

모니터는 `/smartcart/uwb/dual/status`, `/scan`만 구독한다.
선택적으로 `--tracking-status-topic /smartcart/tracking/status`를 주면 위치 없이 추적 상태만 확인한다.
발행자·서비스·모터·TF·직렬 접근은 없다. 자동 기록도 없다.

수동 시험은 세 센서 각각 다음 순서를 실제 관측과 대조한다.

1. 해당 센서가 fresh가 된 뒤 분리 직전에 `unplug uwb_left`를 입력한다.
2. 해당 USB를 실제로 분리하고 수명 초과로 `awaiting_replug`가 될 때까지 기다린다.
3. 연결 직전에 `replug uwb_left`를 입력하고 같은 장치를 같은 물리 USB 슬롯에 다시 연결한다.
4. 새 관측으로 fresh가 되고 수동 시험이 완료되는지 확인한다. `uwb_right`, `lidar_center`도 반복한다.

표시는 표준오류에 상태가 바뀔 때만 출력된다. 단절을 선언하기만 하거나 자연 시간 초과·재생만으로는
물리 분리 시험을 완료하지 않는다. 선언은 작업자의 기록이며 물리 조작을 자동 입증하지 않는다.
LiDAR 드라이버가 단절 후 종료된다면 같은 포트·프레임 설정으로 사용자가 다시 실행한다.
USB 식별값은 location도 포함하므로 다른 슬롯으로 옮기면 식별 대조가 실패할 수 있다.
검수 시간이 끝나기 전 Ctrl+C하면 중단으로 처리한다.

보고서는 센서별 신선도·ID/체크섬 오류·수신 간격·수명·단절/복구 횟수,
좌우 HOST_RX 시간차, LiDAR 도착과 UWB 쌍의 시간차, 스캔 소요 시간을 집계한다.
최근 최대 2048개의 p50/p95/max와 전체 관측의 max_observed를 구분한다.
기본 시간차 한도는 설정 JSON을 따른다. 보고서에 실제 적용한 한도가 함께 나온다.
HOST_RX는 같은 호스트의 소프트웨어 수신 시각이며 센서 하드웨어 동기화·RF 지연의 증명이 아니다.
다른 PC의 monotonic 시각, ROS header 시각과 섞지 않는다.

검수 완료는 장치/장착 준비·세 수동 단절/복구·현재 유효 수신·시간차 한도를 모두 요구한다.
항상 `physical_acceptance=false`, `drive_authorized=false`다. 실물 최종 승인과 주행 허가는 별도다.
종료 코드는 0(명령 정상 또는 모니터 조건 충족), 2(입력/실행 오류),
3(준비/검수 미완료), 130(사용자 중단)이다. `check`의 0은 설정 전제만 확인한 결과다.

## 센서 없는 재생

```bash
python3 field_audit.py replay --input samples/field_acceptance.synthetic.jsonl --output synthetic-field-report-01.json
```

묶음에는 175개 이벤트의 합성 원시 입력 하나만 포함한다. C 파서·수명·오류·복구 집계를 재현하며
위치·범위·각도·광선·태그 주소를 보고서에 남기지 않는다. `source_mode=replay`이며 검수 완료는 false다.
원본 실제 캡처와 개인 로컬 설정은 배포하지 않는다.

보고서 저장은 `--output`을 명시했을 때만 새 로컬 파일(권한 0600, 최대 1MiB)에 수행한다.
기존 파일을 덮어쓰지 않으며 포트·원문 USB 식별자·증빙 원문도 보고서에서 제외한다.
식별자 해시는 익명화 보장이 아니다. 실제 기록은 목적·접근자·보유기간을 정해 검수 후 정리한다.
