# SmartCart 통합 센싱 · Raspberry Pi 4

좌우 BU04 + 중앙 LiDAR 대상 추적의 새 ROS 2·GUI 연결과 실물 없는 통합 재생은
[추적 런타임 안내](../docs/SMARTCART_TRACKING_RUNTIME.md)를 따른다. 아래 기본 실행은 기존
단일 ESP32 진단 모니터이며, 새 추적은 명시적인 `--tracking-mode ros` 또는 `--mode replay`로 켠다.

이 디렉터리는 원본 **SmartCart LiDAR System v1.0.1**을 복사한 뒤 LiDAR와 ESP32 UWB 진단을 함께 표시하도록 확장한 작업본입니다. 연결된 원본 폴더와 상위 프로젝트의 ESP32 펌웨어·PC 디버거는 수정하지 않습니다. 원본 LiDAR 뷰어의 안내는 개발 저장소의 `smartcart/README.md`, 통합 실행은 이 문서를 기준으로 합니다.

실제 탑재 대상은 기존 **Raspberry Pi 4 / Ubuntu 22.04 / ROS 2 Humble / RPLIDAR C1 / `/scan` / `~/smartcart_ws`**입니다. 현재 Pi가 같은 네트워크에 연결되어 있지 않으므로 PC에서 프로그램과 릴리스를 먼저 준비한 뒤 나중에 다운로드·복사하여 설치합니다. **현재 실물 Pi 원격 설치·USB 센서 통합 운전은 검증하지 않았습니다.**

OS·ROS·드라이버를 업그레이드하거나 UWB 장치 설정을 바꾸는 설치기는 없습니다. Jazzy가 이미 설치된 별도 장비에서는 실행기의 `SENSING_ROS_DISTRO=jazzy`로 선택할 수 있지만, 현재 사용자의 탑재 대상과 검수 기준은 Humble입니다. Humble 장비를 Jazzy로 자동 전환하지 않습니다.

## 연결과 안전 범위

```text
RPLIDAR C1 ─ 별도 USB ─ Raspberry Pi 4
                           │
                    기존 sllidar_ros2 ─ /scan
                           │                 │
BU03 Tag ─ UWB 무선 ─ BU04 Base              │
                       UART2_TX ─ ESP32 GPIO18
                                   │         │
                              ESP32 USB ─ sensing.py
                             C_KEY_DIAG_V3/V2   │
                                        ROS 2 + HTTP/SSE
                                               │
                                  SSH 터널 ─ 노트북 GUI
```

- BU03 태그를 Pi에 USB로 연결해야 하는 구조가 아닙니다. BU04 **UART2_TX는 ESP32 GPIO18 한 곳에만** 연결하며 공통 접지를 사용합니다. 다른 모듈의 TX를 이 입력에 병렬 연결하지 않습니다.
- Pi가 읽는 USB 장치는 ESP32의 텍스트 진단 포트입니다. **BU04 자체 USB 데이터 포트는 이진 프레임이므로 이 텍스트 파서에 직접 연결하지 않습니다.** LiDAR USB는 기존 드라이버 한 개가 소유합니다.
- ESP32 시리얼은 통합 서버 한 프로세스만 엽니다. PC 디버거·시리얼 모니터·플래시 도구와 동시에 열지 않습니다. ROS로 UWB를 받는 두 번째 뷰어는 `--uwb-mode ros`를 사용합니다.
- 기존 `LOCKED`, `UNLOCKED`, `WELCOME`, DIP ID 등은 중국 예제의 **도어록 진단 상태**입니다. 스마트카트의 구동 허가·모터 상태로 해석하거나 모터에 연결하지 않습니다. 이 프로그램에는 `/cmd_vel`, GPIO/PWM, 잠금 우회, UWB AT 쓰기, 원격 설정 API가 없습니다.
- TFT LVSN-TFT-2.4의 VCC와 모든 핀에는 5V를 연결하지 않습니다. VCC는 3.3V입니다. ESP32에 외부 5V를 공급하기 전 PC USB를 분리해 역급전을 방지합니다. Pi USB를 이용한 최종 전원 구성도 별도로 검수해야 합니다.
- `AT+GETVER`, `AT+GETCFG`, `AT+GETDEV` 백업 없이 BU03/BU04 설정을 바꾸지 않습니다. 동결된 ESP32 보정값과 고장 시 안전 잠금 로직을 그대로 사용합니다.

## 데이터 계약

| 입력·출력 | 형식 | 처리 |
|---|---|---|
| LiDAR 입력 `/scan` | `sensor_msgs/msg/LaserScan` | 기존 ROS 드라이버 구독. SensorData QoS, 거리 배열과 원래 각도 인덱스 보존 |
| ESP32 입력 | USB 115200 baud, `C_KEY_DIAG_V3` / `V2` | 프레임 길이·필드·수치 검증. 부팅 로그와 잘못된 입력은 위치로 사용하지 않음 |
| UWB ROS 출력 `/smartcart/uwb` | `std_msgs/msg/String`의 JSON | 신원·상태·거리·좌표·진단과 출처 전달 |
| UWB 위치 `/smartcart/uwb/position` | `geometry_msgs/msg/PointStamped` | 유효하고 최신인 위치만 발행. TF 자동 등록·센서 융합은 수행하지 않음 |
| 상태 `/smartcart/sensing/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | LiDAR·UWB 연결 및 입력 유효성 상태 |
| GUI | `http://127.0.0.1:8765` | 같은 출처 정적 자산 + 읽기 전용 JSON/SSE |

LiDAR 화면 전송은 1/2 Hz, 별도 UWB SSE(`/api/uwb/events?hz=10`)와 UWB ROS 발행은 최대 10 Hz입니다. 실제 센서 입력은 가능한 주기로 계속 받습니다. 화면 속도와 ROS 발행 제한으로 LiDAR RPM이나 ESP32 진단 주기를 바꾸지 않습니다. LiDAR의 수신 중단 기준은 기본 1.5초, UWB는 기본 0.5초입니다. 통신 중단 시 이전 위치를 최신 위치처럼 표시하거나 합성 데모로 자동 대체하지 않습니다. HTTP 200만으로 센서 정상 동작을 판단하지 않습니다.

ROS 카트 좌표는 **+X 전방, +Y 왼쪽**입니다. 기존 UWB 진단의 +Y 전방 / +X 오른쪽 좌표를 카트 축으로 변환합니다. UWB 장착 yaw·X/Y의 초기 0은 측정된 교정값이 아닙니다. 실물 장착을 측정한 뒤 `SENSING_UWB_YAW`(deg), `SENSING_UWB_X`·`Y`(m)를 사용합니다. 이는 Pi에서 표시·발행하는 강체 변환이며 ESP32의 거리·각도 보정값을 수정하지 않습니다. LiDAR 장착값은 [보정 문서](docs/CALIBRATION.md)에 따라 GUI에서 별도로 지정합니다. 두 센서의 시간 동기화·가림 처리·추종 제어·SLAM은 이 버전에 포함되지 않습니다.

## 하드웨어 없는 실행

저장소의 `smartcart` 디렉터리에서:

```powershell
python sensing.py serve --mode demo
```

또는 Python 3.10 이상이 있는 Linux에서:

```bash
python3 sensing.py serve --mode demo
```

브라우저에서 `http://127.0.0.1:8765`를 엽니다. LiDAR와 UWB가 합성 데모임을 화면에서 확인합니다. 데모 성공은 Pi·ROS·USB·실측 성공을 뜻하지 않습니다. 웹 런타임에는 npm·CDN·별도 GUI 프레임워크가 필요 없습니다.

## PC에서 Pi용 릴리스 준비

Pi와 연결하지 않고 PC에서 진행할 수 있습니다. 제공된 최종 릴리스를 다운로드하거나, 소스에서 아래 명령으로 새 릴리스를 만듭니다.

프로젝트 루트의 PowerShell에서:

```powershell
python .\smartcart\scripts\package-sensing.py --output .\build\verification\smartcart\smartcart-sensing-v0.1.0.zip
Get-Content .\build\verification\smartcart\smartcart-sensing-v0.1.0.zip.sha256
```

이 패키지는 런타임 코드·웹 자산·ROS 패키지 메타데이터·실행기만 허용 목록으로 묶습니다. 기존 파일을 덮어쓰지 않으므로 재생성 시 새 이름을 사용합니다. `.env`, 로컬 `sensing.env`, SSH 키, `.git`, 가상환경, USB 로그, 기록 파일, 개발 테스트 결과는 포함하지 않습니다. 복사본에 남아 있는 기존 `MANIFEST.sha256`는 원본 v1.0.1 참고 자료입니다. 통합 ZIP에는 새 **`RELEASE.sha256`** 목록을 넣습니다. ZIP 옆에는 수동 설치기와 ZIP·설치기 양쪽의 해시를 담은 `.zip.sha256`를 생성합니다.

## 우선 절차: 다운로드·USB 복사 후 Pi에 설치

다음 **세 파일**을 같은 폴더에 보관하고, 나중에 USB 저장장치·파일 복사·로컬 다운로드 중 가능한 방식으로 Pi의 다운로드 폴더 등 현재 사용자 영역에 옮깁니다. 실제 제공된 릴리스 이름이 다르면 세 파일 이름을 함께 맞춰 사용합니다.

- `smartcart-sensing-v0.1.0.zip`
- `smartcart-sensing-v0.1.0-install.py`
- `smartcart-sensing-v0.1.0.zip.sha256`

Pi의 일반 사용자 터미널에서 세 파일을 복사한 폴더로 이동한 뒤:

```bash
# ZIP과 설치기 모두 OK인지 확인합니다. 하나라도 실패하면 실행하지 않습니다.
if sha256sum -c smartcart-sensing-v0.1.0.zip.sha256; then
  read -r SHA256 _ < smartcart-sensing-v0.1.0.zip.sha256
  /usr/bin/python3 smartcart-sensing-v0.1.0-install.py \
    --archive smartcart-sensing-v0.1.0.zip --sha256 "$SHA256"
else
  printf '%s\n' '복사본 검증 실패: 설치하지 않았습니다.' >&2
fi
```

외부 해시는 신뢰하는 PC에서 만든 원본과 복사본의 일치 여부를 확인하는 용도이며, 임의 출처의 실행 파일을 신뢰하게 만드는 서명은 아닙니다. 설치기는 ZIP 전체 SHA-256과 내부 모든 파일 해시, 경로·특수 파일·파일 수·해제 크기 상한을 다시 검사합니다. ZIP을 임의로 먼저 풀거나 검증을 생략하지 않습니다.

현재 사용자의 **`~/SmartCart-Sensing`**에만 새로 설치합니다. 기존 `~/SmartCart-Radar`, `~/smartcart_ws`와 이미 있는 `~/SmartCart-Sensing`을 덮어쓰지 않습니다. 같은 설치 폴더가 있으면 내용을 검토하여 이전 버전 이름으로 보존한 뒤 재시도하십시오. `sudo`, 패키지 설치, 서비스 등록·시작, LiDAR 드라이버 실행은 자동 수행하지 않습니다. 설치 후 아래의 Pi 실측 실행과 현장 수락 검사를 진행합니다. Pi가 외부 네트워크에 연결되지 않아도 파일 설치는 가능하며, ROS·pyserial 등 기존 런타임 준비는 별도입니다.

## 대안: 연결 가능할 때 SSH 전송과 설치

Pi가 나중에 같은 네트워크 또는 정상적인 SSH 경로에 연결되면 기존 계정 **`capstone@SUDO`**를 사용할 수 있습니다. `SUDO`는 Pi 호스트명이며 이 명령은 관리자 권한 `sudo`를 뜻하지 않습니다. 이름이 해석되지 않으면 Pi에서 `hostname -I`로 확인한 **현재 주소**를 사용합니다. 이전 실험 IP를 추정해 접속하지 않습니다.

로컬 OpenSSH의 `ssh`와 `scp`, Pi의 `/usr/bin/python3`가 필요합니다. SSH 비밀번호나 개인키를 소스·환경 예제에 넣지 않습니다. 처음 연결하는 장비는 별도 SSH 창에서 장비의 호스트 키 지문을 대조하고 키/에이전트 인증을 먼저 확인합니다. 배포기는 `BatchMode=yes`, `StrictHostKeyChecking=yes`로 동작하므로 등록되지 않은 호스트나 비밀번호 입력이 필요한 연결에서 중단합니다.

PowerShell에서 실제 값을 지정한 뒤 먼저 드라이런합니다:

```powershell
$PiTarget = 'capstone@SUDO' # 이 호스트가 실제로 연결 가능할 때. 필요하면 현재 Pi IP로 변경합니다.
$Archive = '.\build\verification\smartcart\smartcart-sensing-v0.1.0.zip'
$Digest = (Get-FileHash -Algorithm SHA256 -LiteralPath $Archive).Hash.ToLowerInvariant()
python .\smartcart\scripts\deploy-sensing.py --target $PiTarget --archive $Archive --sha256 $Digest --dry-run
# 대상·해시를 확인한 뒤 전송 및 설치. SSH 포트가 다르면 --ssh-port를 명시합니다.
python .\smartcart\scripts\deploy-sensing.py --target $PiTarget --archive $Archive --sha256 $Digest
```

이 대안도 위와 같은 설치 검증·기존 폴더 보존 정책을 적용합니다. 송신용 임시 ZIP과 설치기는 Pi 홈에 무작위 이름으로 남으며 설치 완료 후 출력된 정확한 파일명만 직접 제거할 수 있습니다. 현재 연결되지 않은 Pi에 대한 전송·설치가 완료되었다는 뜻은 아닙니다.

## Pi에서 실측 실행

Pi의 로컬 터미널이나 연결이 가능해진 SSH 터미널에서 OS/ROS와 장치를 먼저 확인합니다:

```bash
cat /etc/os-release
uname -m
ls -l /dev/serial/by-id/
cd ~/SmartCart-Sensing
bash scripts/doctor-sensing.sh --require-live
```

진단 실패 원인을 해결하기 전 데모를 실측처럼 사용하지 않습니다. Humble 실측 모드는 기존 ROS Python 모듈과 `pyserial`이 필요합니다. `python3-serial`이 없다면 설치 명령을 먼저 검토한 후 운영자가 실행합니다:

```bash
sudo apt-get install python3-serial
```

`rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `diagnostic_msgs`는 사용 중인 ROS 배포판과 `/usr/bin/python3`에 맞아야 합니다. Windows·가상환경의 Python을 Pi ROS 노드 실행에 사용하지 않습니다. 시리얼 권한 오류는 `id`, `ls -l`과 `dialout` 그룹을 확인하고, 필요한 계정 권한만 운영자가 부여합니다. 장치 전체 `chmod 666`이나 루트 실행으로 우회하지 않습니다.

LiDAR 드라이버는 **이미 실행 중이면 추가로 시작하지 않습니다.** 새로 실행하는 경우 기존 성공한 드라이버 설정과 장치 경로를 유지합니다. 예:

```bash
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# 기존 ROS_DOMAIN_ID, RMW_IMPLEMENTATION이 있으면 같은 값을 유지합니다.
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

다른 Pi 터미널에서 ESP32 **CH343 진단 USB**의 정확한 링크를 선택합니다. `/dev/ttyUSB0` 번호만 보고 장치를 추정하지 않습니다:

```bash
cd ~/SmartCart-Sensing
export SENSING_SERIAL_PORT='/dev/serial/by-id/실제_ESP32_링크로_변경'
# 기본 humble, ~/smartcart_ws, /scan, /smartcart/uwb, 8765
bash scripts/start-sensing.sh
```

링크가 없거나 접근할 수 없으면 시작이 실패합니다. 포트를 지정하지 않고 임의 USB 장치를 자동 선택하지 않습니다. 한글 자리표시자는 실제 ASCII 장치 링크로 바꾸십시오. 기존 LiDAR 노드와 통합 서버는 같은 `ROS_LOCALHOST_ONLY`, `ROS_DOMAIN_ID`, ROS 배포판 및 RMW 환경을 사용해야 합니다. 드라이버가 LAN ROS를 의도적으로 사용 중이면 운영자의 실제 설정에 맞춰 기본값을 검토합니다.

같은 실행을 직접 구성하려면:

```bash
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
/usr/bin/python3 sensing.py serve --mode ros --uwb-mode serial \
  --serial-port "$SENSING_SERIAL_PORT" --baud 115200 --topic /scan \
  --uwb-topic /smartcart/uwb --host 127.0.0.1 --port 8765
```

두 번째 읽기 전용 관측 프로그램이 UWB ROS 토픽을 이용할 때는 `--uwb-mode ros`로 실행하고 별도 HTTP 포트를 지정합니다. ESP32 USB를 다시 열거나 같은 토픽을 재발행하지 않는 모드입니다.

Pi에 브라우저가 있는 로컬 데스크톱 환경이면 Pi 자체에서 `http://127.0.0.1:8765`를 열 수 있습니다. 노트북에서는 나중에 SSH 연결이 가능해졌을 때 새 SSH 창에서 터널을 유지합니다:

```powershell
ssh -N -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 $PiTarget
```

브라우저에서 `http://127.0.0.1:8765`를 엽니다. 기본 HTTP는 Pi 루프백에만 열며 방화벽·라우터의 8765 포트를 공개할 필요가 없습니다. SSH 포트가 다르면 이 명령에도 `-p`를 지정합니다.

## ROS 확인과 선택 설치

서버와 같은 ROS 환경을 source한 터미널에서:

```bash
ros2 topic info /scan --verbose
ros2 topic info /smartcart/uwb --verbose
ros2 topic echo /smartcart/uwb std_msgs/msg/String --once --qos-reliability best_effort
ros2 topic echo /smartcart/uwb/position geometry_msgs/msg/PointStamped --once --qos-reliability best_effort
ros2 topic echo /smartcart/sensing/diagnostics diagnostic_msgs/msg/DiagnosticArray --once --qos-reliability best_effort
```

위치는 최신·유효 UWB가 있을 때만 발행되므로 태그가 꺼져 있으면 position의 `--once`가 기다릴 수 있습니다. 종료는 Ctrl+C입니다. 토픽이 있다는 사실만으로 데이터가 최신이라는 뜻은 아닙니다. 원본 `/scan`의 발행 노드·수신률을 함께 확인하십시오. **`LaserScan` 메시지 자체에는 실물·시뮬레이터를 증명하는 출처 정보가 없으므로 ROS 수신 상태만으로 실물 입력을 확정할 수 없습니다.** DDS 전송 테스트의 스캔·UWB 값은 가상 입력이며 실제 C1·BU03/BU04·ESP32 검수를 대체하지 않습니다.

소스 실행에는 `colcon build`가 필요하지 않습니다. ROS 패키지로 등록하려면 기존 `~/smartcart_ws`와 별개인 통합 워크스페이스를 사용합니다. 해당 새 경로가 다른 프로젝트와 겹치지 않는지 확인한 뒤:

```bash
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
mkdir -p ~/smartcart_sensing_ws/src
ln -s "$HOME/SmartCart-Sensing" "$HOME/smartcart_sensing_ws/src/smartcart_sensing"
cd ~/smartcart_sensing_ws
colcon build --packages-select smartcart_sensing --symlink-install
source install/setup.bash
ros2 launch smartcart_sensing sensing.launch.py \
  serial_port:="$SENSING_SERIAL_PORT"
```

launch는 serial_port를 필수로 받고 HTTP는 `127.0.0.1`에 바인딩합니다. 선택 인수는 `topic`, `uwb_topic`, `port`, `uwb_yaw`, `uwb_x`, `uwb_y`입니다. **제공 launch는 기존 `ROS_LOCALHOST_ONLY` 값을 보존하며, 값이 없으면 1을 사용합니다. LiDAR 드라이버와 같은 값이어야 합니다.** 이미 실행 중인 드라이버의 기존 환경을 확인하고, 설정이 다른 상태에서 통합 노드만 성공했다고 기록하지 않습니다. `ROS_DOMAIN_ID`는 기존 값을 유지합니다. 기존 드라이버의 빌드·설정은 변경하지 않습니다.

## 선택: 현재 계정의 systemd 서비스

먼저 수동 실측 수락 검사를 마친 뒤 사용합니다. 자동 등록하지 않습니다. 아래는 **Pi의 일반 SSH 계정**에서 직접 실행할 명령입니다:

```bash
mkdir -p ~/.config/smartcart-sensing ~/.config/systemd/user
install -m 600 deploy/sensing.env.example ~/.config/smartcart-sensing/sensing.env
# 편집기에서 SENSING_SERIAL_PORT를 실제 ESP32 링크로 수정합니다.
nano ~/.config/smartcart-sensing/sensing.env
install -m 600 deploy/smartcart-sensing.service ~/.config/systemd/user/smartcart-sensing.service
systemctl --user daemon-reload
systemctl --user start smartcart-sensing.service
systemctl --user status smartcart-sensing.service
```

기존 수동 서버를 먼저 종료해 시리얼·HTTP 중복 점유를 피합니다. 환경 파일에는 장치 경로·ROS 설정만 두고 셸 명령·비밀번호·키를 넣지 않습니다. 이 파일은 `source`하지 않으며 systemd가 환경 항목으로 읽습니다. 사용자 unit의 `EnvironmentFile`은 `~`·`$HOME` 같은 셸 확장을 하지 않으므로 워크스페이스를 지정할 때 실제 절대 경로를 사용합니다.

서비스는 현재 계정으로만 동작하며, 루프백 HTTP·읽기 전용 홈·새 권한 획득 금지·재시작 횟수 제한을 적용합니다. ROS 런타임 로그는 사용자 런타임 디렉터리에 둡니다. LiDAR 드라이버는 별도 운영하며 이 서비스가 시작·종료하지 않습니다. user namespace 제한 등으로 보안 설정을 적용할 수 없는 장비에서는 상태 오류를 확인하고 수동 실행으로 돌아갑니다. 제한을 임의 해제하여 성공으로 기록하지 않습니다.

로그 확인·중지:

```bash
journalctl --user -u smartcart-sensing.service -n 50 --no-pager
systemctl --user stop smartcart-sensing.service
```

로그인 시 자동 실행을 원하는 경우에만 수락 검사 후 `systemctl --user enable smartcart-sensing.service`를 실행합니다. 로그아웃 후 상시 실행을 위한 linger 설정은 시스템 운영 정책을 별도로 확인해야 하며 자동 변경하지 않습니다. 서비스 중지는 카트 모터 정지나 기존 ESP32 도어록 출력 변경이 아닙니다.

## 현장 수락 검사

아래 항목은 실제 Pi·USB·센서를 사용한 기록이 있어야 완료입니다. 데모·자동 테스트 결과와 구분합니다.

1. 릴리스 SHA-256, OS/ROS 버전, Pi 모델, 두 USB by-id 경로, LiDAR/BU04/ESP32 장착 사진을 기록합니다.
2. GUI의 출처가 실제 ROS·ESP32이고 두 입력의 수신률·최종 수신 시간이 갱신되는지 확인합니다. 데모 배지나 합성 좌표를 실측 결과로 사용하지 않습니다.
3. LiDAR 전·좌·우의 알려진 표적으로 방향과 거리를, UWB 0도·±30도·±45도와 1/2/3m 표적으로 장착 좌표를 검수합니다. 원본의 동결 보정값을 보기 좋게 바꾸지 않습니다.
4. 태그 전원 차단, ESP32 USB 분리, LiDAR 드라이버 중단을 각각 시험합니다. 해당 센서의 점·거리·최신 판정이 무효화되고 다른 센서 출처와 구분되는지 확인합니다. 현재 ESP32의 안전 잠금 유지도 실물로 관찰합니다.
5. DIP ID 불일치/복구, 잘못된 텍스트 포트 선택, 시리얼 중복 실행, Pi 재시작 후 잘못된 LIVE 표시가 없는지 확인합니다. 잠금 로직을 우회하지 않습니다.
6. 실제 카트 장착값으로 최소 30분 운전하며 재연결, 발열, 전원·공통 접지, CPU/RAM, 수신 지연과 USB 재열거를 기록합니다. 구동륜은 고정하고 별도 하드웨어 정지 수단을 준비합니다.
7. 통합 센싱 수락과 기존 도어록 과제 수락을 구분합니다. 기존 정식 보고서 대기 항목은 개발 저장소의 `docs/CONTEST_ACCEPTANCE_STATUS.md`에 그대로 남아 있습니다. 이 문서는 센싱 런타임 ZIP에 포함되지 않습니다.

센서 데이터는 최신값만 메모리에 두며 기본 자동 기록·외부 클라우드 전송·회원 DB·추적 분석을 추가하지 않습니다. 태그 ID와 위치를 특정 사람과 연결하면 개인정보가 될 수 있으므로 실제 사용 전 목적·접근자·보관기간을 정해야 합니다. 파일 내보내기·ROS bag 등 별도 기록은 운영자가 선택한 범위만 수행하고 공유 전 태그 ID·주변 공간 정보·로컬 경로를 검토하십시오.

## 개발 검증

```bash
python3 -m unittest discover -s tests -v
node tests/test_frontend.cjs
node tests/test_contours.cjs
node tests/test_sensing_frontend.cjs
```

`tests/test_deploy_sensing.py`는 릴리스 해시·수동 복사용 설치기·비밀파일 제외·ZIP 경로 이동/특수파일/크기 제한·SSH 인수 검증과 실제 Bash 구문을 검사합니다. 현재 실제 Pi에 대한 원격 배포, systemd 사용자 권한, ARM64 ROS 런타임, 실물 장치의 정확도와 전원 안정성은 검증하지 않았습니다. ROS DDS 전송 검증과 합성 GUI 테스트는 가상 입력에 대한 소프트웨어 검사입니다. 실험 종료 시 통합 서버를 먼저 종료하고, 별도로 실행한 LiDAR 드라이버도 종료합니다.
# 좌우 BU04 신규 수집 묶음

2026-10-23 추가: D5의 좌우 BU04 데이터 USB를 독립적으로 읽는 수집기·공통 관측·
장착 설정·재생 시험기는 [별도 안내](../docs/SMARTCART_DUAL_UWB.md)를 따른다.
기존 단일 BU04+ESP32 진단과 아래 GUI 실행 절차는 유지한다. 신규 출력도 구동 허가가 아니다.
좌우 좌표·시간 정합과 태그 위치 추정·합성 재생은 [위치 추정 안내](../docs/SMARTCART_UWB_ESTIMATION.md)를 따른다.
중앙 LiDAR 클러스터와 UWB 기반 대상 연관·추적은 [대상 추적 안내](../docs/SMARTCART_TARGET_TRACKING.md)를 따른다.

