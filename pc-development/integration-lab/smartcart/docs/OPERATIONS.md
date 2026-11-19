# 설치·기본 운영

이 문서는 보존된 LiDAR 전용 진단 절차다. 현재 D5의 LiDAR·좌우 UWB 통합 실측은
[통합 README](../README.md)의 `start-live.sh`와 운영 런북을 따른다.
현재 시작 도구에는 아래 LiDAR 전용 `--host` 인수를 넘기지 않는다.

## 1. 구성

```text
RPLIDAR C1 ─ USB ─ Raspberry Pi 4 / Ubuntu 22.04 / ROS 2 Humble
                       │
                       ├ sllidar_ros2 ─ /scan (기존 약 10 Hz)
                       │                  │
                       │               radar.py
                       │                  │ HTTP / SSE, 표시 1 또는 2 Hz
                       └──────────────────┴── 노트북 브라우저 / ASCII CLI
```

Pi의 이미 설치된 드라이버와 워크스페이스를 그대로 사용합니다. 레이더는 `/dev/ttyUSB0`를 직접 열거나 C1 모터 속도를 바꾸지 않습니다. Windows에는 ROS를 설치하지 않아도 됩니다. Pi와 브라우저가 같은 신뢰할 수 있는 LAN에 있어야 합니다. 다른 네트워크에서는 SSH 터널을 사용합니다.

## 2. ZIP 전달

현재 Pi에 연결된 SSH 터미널에서:

```bash
hostname -I
```

Windows PowerShell의 ZIP 다운로드 폴더에서, `$PiIP`를 방금 확인한 주소로 바꿉니다. 아래 주소는 이전 운영값의 **예시**입니다.

```powershell
$PiIP = "192.168.43.212"
scp .\SmartCart-Radar-v1.0.1.zip "capstone@${PiIP}:~/"
ssh "capstone@$PiIP"
```

Pi에서:

```bash
cd ~
unzip SmartCart-Radar-v1.0.1.zip
cd ~/SmartCart-Radar
ls
```

기존 같은 이름의 폴더가 있다면 먼저 백업하거나 별도 위치에 풉니다. `unzip`이 없으면 Python 표준 라이브러리로 풀 수 있습니다.

```bash
cd ~
python3 -m zipfile -e SmartCart-Radar-v1.0.1.zip .
```

## 3. 실측 모드

### SSH 터미널 1: LiDAR

이미 실행 중이라면 추가 실행하지 않습니다. 드라이버를 실행한 창은 유지합니다.

```bash
cd ~/smartcart_ws
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

### SSH 터미널 2: 레이더

```bash
cd ~/SmartCart-Radar
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
/usr/bin/python3 radar.py serve --mode ros --host 0.0.0.0 --port 8765 --topic /scan
```

이전 LiDAR 전용 진단은 환경을 직접 source하고 `/usr/bin/python3`를 선택합니다. 가상환경의 Python과 ROS 패키지 ABI가 다른 문제를 피하기 위한 선택입니다. 위 명령과 같은 수동 실행은 다음과 같습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
cd ~/SmartCart-Radar
/usr/bin/python3 radar.py serve --mode ros --host 0.0.0.0 --port 8765 --topic /scan
```

노트북 브라우저에 `http://<현재 Pi IP>:8765`를 입력합니다. 웹 페이지가 먼저 열려도 드라이버가 스캔을 발행하기 전까지는 `WAITING`입니다. 이후 `LIVE`과 실제 서버 수신률을 확인합니다.

### 방화벽을 사용하는 경우

우선 Windows PowerShell에서 TCP 도달 여부를 확인합니다.

```powershell
Test-NetConnection $PiIP -Port 8765
```

UFW 등을 무조건 끄거나 공용 인터넷에 포트를 공개하지 마십시오. 필요한 경우 신뢰하는 노트북 IP에만 TCP 8765 접근을 허용합니다. 핫스팟의 단말 간 격리 여부, 현재 Pi IP, 같은 네트워크인지도 확인합니다.

### SSH 터널 방식

Pi 서버를 기본값인 루프백에만 바인딩합니다.

```bash
cd ~/SmartCart-Radar
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
/usr/bin/python3 radar.py serve --mode ros --host 127.0.0.1 --port 8765 --topic /scan
```

Windows PowerShell 새 창에서:

```powershell
$PiIP = "192.168.43.212" # 현재 주소로 변경
ssh -N -L 8765:127.0.0.1:8765 "capstone@$PiIP"
```

브라우저에서는 `http://127.0.0.1:8765`를 엽니다. 터널 창도 유지해야 합니다. 서버는 인증·TLS를 구현하지 않은 실험용 읽기 전용 서버이므로, 공용 네트워크에서는 이 방식을 사용합니다.

## 4. 화면 사용

`SOURCE`, `UPDATE`, `RANGE`는 오른쪽 STATUS 패널에 있습니다. 파일 재생, PNG/JSON 저장, 전체 화면, 장착 보정 및 진단은 하단 `설정`에서 엽니다. 모바일에서는 패널이 레이더 아래로 이동하며 세로 스크롤을 사용합니다.


| 항목 | 동작 |
|---|---|
| ROS 서버 | 현재 페이지를 제공하는 서버의 입력을 표시합니다. 서버가 demo/replay이면 출처가 별도로 표시됩니다. |
| 첨부 스캔 | 원본 첨부 파일의 128개 항목만 표시합니다. 0.25 m로 자동 확대되는 정적 부분 스캔입니다. |
| 합성 데모 | 가상의 방·기둥·물체를 ray casting한 UI 테스트입니다. 실측이 아닙니다. |
| 파일 재생 | JSON/JSONL을 브라우저에서 직접 읽습니다. 서버로 업로드하지 않습니다. |
| 반경 | 카트 중심 기준 0.25/0.5/1/2/3/5 m입니다. 실제 센서 최대거리는 바꾸지 않습니다. |
| 표시 갱신 | 1 Hz=1초/회전, 2 Hz=0.5초/회전입니다. ROS 입력 수신률과 독립입니다. |
| 화면 정지 | 표시 프레임만 고정합니다. `/scan` 수신·LiDAR·구동 모터를 정지하지 않습니다. |
| 점 클릭 | 스캔 인덱스, 카트 기준 X/Y와 중심 거리를 표시합니다. Esc로 선택을 해제합니다. |
| PNG 저장 | 출처·상태와 거리 눈금이 있는 3개 Canvas 레이어를 합쳐 저장합니다. |
| 스캔 JSON | 전체 수신 배열과 표시 설정을 저장합니다. 5 m 밖 항목도 유지합니다. |
| 전체 화면 | 브라우저 전체 화면입니다. 별도 네이티브 앱 설치 파일은 아닙니다. |

정상 메시지의 경우 센서 유효 범위 밖 항목과 `null`은 표시에서 제외됩니다. 이 값들이 제거되더라도 나머지 항목의 원래 배열 인덱스와 각도를 유지합니다. 빈 영역을 채우지 않으며 인접점 간 보수적인 거리 조건을 만족하는 구간만 윤곽선으로 연결합니다. [윤곽 처리](CONTOURS.md)를 참조하십시오.

센서 입력이 1.5초 넘게 오지 않으면 표시점을 숨기고 방향별 거리도 `—`로 바꿉니다. `OFFLINE`과 `STALE`을 구분합니다. 데모로 자동 전환하지 않습니다. ROS 헤더와 서버 시계의 동기화를 가정하지 않으며, 이 경과 시간은 **서버 수신 이후 시간**이지 정확한 센서 노출→화면 지연이 아닙니다.

## 5. CLI

서버가 동작 중인 Pi에서:

```bash
cd ~/SmartCart-Radar
python3 radar.py cli --url http://127.0.0.1:8765 --hz 2
```

Windows에서:

```powershell
py -3 radar.py cli --url "http://${PiIP}:8765" --hz 2
```

보정값은 웹 브라우저 설정과 별개입니다.

```bash
python3 radar.py cli --radius 5 --yaw 90 --x 0.20 --y 0.00
```

`--once`는 한 번 출력하고 종료합니다. CLI에서 `^`는 카트 전방, `*`는 유효 반사점입니다. CLI는 1/2 Hz ASCII 스캔 갱신이며 브라우저처럼 부드러운 회전 애니메이션은 하지 않습니다. 작은 터미널에서는 여러 점이 한 문자 셀로 겹칩니다.

## 6. 전체 데이터 저장·재생

ROS 환경이 활성화된 Pi 터미널에서:

```bash
cd ~/SmartCart-Radar
source /opt/ros/humble/setup.bash
source ~/smartcart_ws/install/setup.bash
/usr/bin/python3 radar.py record --seconds 30 --output recordings/test-01.jsonl
```

화면 전송률인 1/2 Hz가 아니라 **이 구독자가 수신한 모든 스캔 메시지**를 기록합니다. 결과에 저장 개수와 기록 큐 드롭 수가 출력됩니다. 기존 파일을 덮어쓰지 않습니다. DDS 전송 손실까지 없음을 보장하는 수치는 아닙니다.

JSON 규격에 없는 Inf/NaN은 `null`로 정규화합니다. 각도별 위치·메타데이터·유한한 거리값은 유지합니다. `range_min`보다 작은 원본 유한 값도 기록에는 남고, 렌더링에서만 제외됩니다. 음의 무한대와 양의 무한대, NaN의 원래 종류를 구별해야 하는 분석에는 원래 ROS bag을 별도로 기록하십시오.

```bash
# 한 번 재생 후 마지막 프레임에서 종료 상태 표시
python3 radar.py serve --mode replay --file recordings/test-01.jsonl

# 반복 재생
python3 radar.py serve --mode replay --file recordings/test-01.jsonl --loop
```

재생 간격은 각 메시지의 `scan_time`을 사용합니다. 원래 ROS 헤더 타임스탬프 사이의 누락·중단 구간을 정확히 재현하는 bag player가 아닙니다. `--replay-hz 10`으로 재생 간격을 명시할 수 있습니다. 서버 파일 한도는 100 MiB, 브라우저 재생 한도는 25 MiB·20,000 프레임입니다.

### CLI 출력 생략 없이 1회 저장

```bash
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --full-length > scan-full.yaml
/usr/bin/python3 radar.py convert scan-full.yaml --output scan-full.json
/usr/bin/python3 radar.py inspect scan-full.json
```

`convert`는 기본 block-style ROS echo YAML의 첫 스캔 또는 JSON/JSONL을 지원합니다. 임의 YAML 프로그램 실행, `--flow-style` 배열, 터미널 프롬프트가 섞인 로그는 지원하지 않습니다. 여러 스캔 변환 결과는 JSONL로 저장되므로 `.jsonl` 확장자를 사용하십시오.

## 7. 종료

브라우저를 닫거나 CLI에서 Ctrl+C를 누르면 해당 뷰어만 종료됩니다. 레이더 서버 창의 Ctrl+C는 서버와 그 서버가 만든 ROS 구독 노드만 종료합니다. **별도 실행 중인 LiDAR 드라이버는 계속 동작합니다.**

실험을 마칠 때는 레이더 서버 종료 → LiDAR 드라이버 창 Ctrl+C → 드라이버 종료 확인 순서로 처리합니다. Pi 전원을 끌 때는:

```bash
sudo poweroff
```

이 프로그램은 구동 모터나 비상정지 회로에 접근하지 않습니다. 실제 카트를 움직이는 실험에는 독립적인 하드웨어 정지 수단이 필요합니다.

## 8. 진단

```bash
cd ~/SmartCart-Radar
bash scripts/doctor.sh
```

| 증상 | 확인 |
|---|---|
| `No module named rclpy` | `/opt/ros/humble/setup.bash` source 여부와 `/usr/bin/python3` 사용 여부를 확인합니다. |
| 화면은 열리지만 스캔 대기 | `ros2 topic list`, `ros2 topic info /scan --verbose`, LiDAR 드라이버 창을 확인합니다. |
| 다른 터미널만 스캔을 받음 | 드라이버와 서버의 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`과 ROS 환경을 비교합니다. 임의의 새 도메인으로 변경하지 않습니다. |
| 포트 연결 안 됨 | `hostname -I`, `ss -ltn '( sport = :8765 )'`, 바인딩 주소·핫스팟 단말 격리를 확인합니다. |
| 원점 부근 점만 보임 | 샘플이 부분 스캔인지, 실제 주변 물체가 5~6 cm에 있는지 구분합니다. 반경 0.25 m로 시각적 확대만 합니다. |
| 전후좌우가 반대 | CALIBRATION 문서대로 알려진 물체로 yaw를 확인합니다. 그림을 보기 좋게 맞추려고 임의로 거리를 바꾸지 않습니다. |
| 로컬 HTML에서 실측 연결 불가 | `file://`는 오프라인 용도입니다. Pi의 HTTP 페이지로 접속합니다. |

서버 API 확인:

```bash
python3 -c 'import json,urllib.request; print(json.load(urllib.request.urlopen("http://127.0.0.1:8765/api/health")))'
```

`state=live`, `source_mode=ros`, 지속 증가하는 스캔 시퀀스와 실제 수신률을 함께 확인합니다. HTTP 200 응답만으로 센서가 정상이라는 뜻은 아닙니다.
