# SmartCart Raspberry Pi 4 운영 런북

Pi 4 · C1 LiDAR · 왼쪽 BU04 · BU03 · 웹 조이스틱과 가상 주행 시험

## 실제 카트 구동 구성과 현재 상태

제공받은 Pi 내보내기 원본에는 실제 GPIO 백엔드를 사용할 수 있는 수동 조종 코드가 있다. 재부팅 후 사용자가 보고한 센싱 실행 경로는 `~/smartcart/SmartCart-D5-v2/smartcart/cart.py`다. 실제 조종 설치 경로와 Pi 파일 해시는 별도 확인이 필요하다. `pi-runtime/pi-export-reference/`는 개인 설정을 제외한 제공 원본 소스 기준본이며 실제 Pi 설치본과의 일치 여부는 미확인이다.

**현재 PC의 `pi-runtime/`은 실제 주행 승인본이 아니다.** 그 안의 `manual.sh`와 `drive.py`는 가상 출력만 허용하고 실제 GPIO를 잠근다. 가져온 기존 수동 서버는 LiDAR·BU04를 표시하지만 센서 무효 시 모터를 잠그는 기능은 없으며, 2초 PWM 응답 감시가 Pi 실물 설치본에 적용됐는지도 확인되지 않았다. 이 상태에서 자동 추종·장애물 회피가 실물에 동작한다고 간주하지 않는다.

실물 구동 전에 좌우 전진 극성, 독립 모터 전원 차단 및 Pi 비상정지 피드백, LiDAR 장착·정지 범위, 단일 BU04 측정·태그 위치 정확도, 센서 이상과 통신 만료 시 정지·재시작 조건을 확인한다. 엔코더가 없어 PWM 응답과 실제 바퀴 움직임은 다르다. 화면 STOP은 PWM 0 요청이다.

PC에서 개발하는 현재 사본은 `pc-development/`, Pi 실행 후보 사본은 `pi-runtime/`이다. 두 폴더는 자동 동기화되지 않으며, Pi 후보에는 장치별 현장 설정·인증서와 빌드 산출물이 없다. Ubuntu ARM64/ROS 2 Humble 빌드와 Pi 배포·실물 시험은 아직 하지 않았다. 원본 Pi 내보내기 ZIP과 장치 설정 백업은 별도로 보존한다.

## 현재 알려진 상태

Pi의 마지막으로 확인된 주소는 `192.168.43.212`다. 현재 주소는 Pi에서 `hostname -I`로 확인한다. Pi·PC·휴대폰은 같은 `192.168.43.X` 사설망에 연결한다.

이전에 실행한 별도 가상 주행 서버는 HTTPS `18084`에서 `MockBackend`를 사용했다. 새 `pi-runtime/manual.sh`는 HTTPS `8081`에서 가상 출력만 사용한다. 둘은 별도 프로그램이며 새 사본은 아직 Pi에 배포되지 않았다. 실측 LiDAR·BU04 값은 8765 센싱 서버에서 읽는다. 같은 포트에서 서버를 중복 실행하지 않는다.

재부팅 후 마지막 보고에서 센싱 서버는 `127.0.0.1:8765`에서 HTTP 200을 반환했고 단일 BU04·ROS 센싱과 실행기 잠금 보유가 확인됐다. 이미 실행 중인 경우 센싱 시작 스크립트를 다시 실행하지 않는다. 과거에는 LiDAR 무효 빔과 BU04 측정 무효·ID 미확인이 관측됐으며, 재부팅 후 센서 측정 유효성과 정확도는 이 서버 상태만으로 확인되지 않는다.

## 주소 확인 및 SSH 접속

Pi에 연결된 PC의 PowerShell에서 접속한다.

```powershell
ssh capstone@192.168.43.212
```

주소가 변경되면 Pi SSH에서 확인한다.

```bash
hostname -I
```

## 센싱 상태 확인 및 시작

LiDAR와 왼쪽 BU04를 Pi에 연결하고 BU03 태그 전원을 켠다. 먼저 별도 Pi SSH 창에서 기존 센싱이 이미 실행 중인지 확인한다.

```bash
ss -ltnp '( sport = :8765 )'
curl -sS --max-time 2 -o /dev/null -w '센싱 HTTP 상태: %{http_code}\n' http://127.0.0.1:8765/api/health
ps -ww -eo pid,ppid,args | grep -E '[c]art.py|[s]martcart_runner|[s]ensing.py|[r]adar.py'
fuser -v /tmp/smartcart-runner-*/live.lock /tmp/smartcart-runner-*/8765.lock
```

센싱 응답이 200이고 `cart.py` 실행 프로세스가 있으면 기존 세션을 사용한다. 포트가 열려 있지 않은 경우에만 다음 명령을 한 번 실행하고 그 터미널을 유지한다.

```bash
bash "$HOME/smartcart/SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh"
```

이 실행은 모터를 구동하지 않는다. 단일 BU04의 현재 설정은 수평거리 가정 시험이며 장착·거리 정확도가 승인된 상태가 아니다.

### 센서 상태 판독

Pi에서 최신 값 다섯 번을 읽는다. 출력에는 태그 위치나 식별번호를 포함하지 않는다.

```bash
python3 - <<'PY'
import json
import time
from urllib.request import urlopen

for index in range(5):
    try:
        with urlopen('http://127.0.0.1:8765/api/frame', timeout=2) as response:
            frame = json.load(response)
        scan = frame.get('scan') or {}
        ranges = scan.get('ranges') or []
        invalid = sum(not isinstance(value, (int, float)) or value != value
                      or value < scan.get('range_min', 0)
                      or value > scan.get('range_max', 0) for value in ranges)
        print('LiDAR:', frame.get('source_mode'), frame.get('state'),
              '나이 ms:', frame.get('age_ms'), '빔:', len(ranges), '무효:', invalid)
        with urlopen('http://127.0.0.1:8765/api/dual-uwb', timeout=2) as response:
            status = json.load(response)
        left = (status.get('sensors') or {}).get('uwb_left') or {}
        print('BU04:', status.get('profile'), left.get('state'),
              'USB:', left.get('connected'), '측정:', left.get('measurement_valid'),
              'ID:', left.get('id_valid'), '사유:', left.get('invalid_reason'))
        with urlopen('http://127.0.0.1:8765/api/tracking', timeout=2) as response:
            tracking = json.load(response)
        track = (tracking.get('result') or {}).get('track') or {}
        print('추적:', tracking.get('state'), track.get('state'),
              '유효:', track.get('valid'), '후보:', track.get('candidate_count'),
              '사유:', track.get('invalid_reason'))
    except Exception as error:
        print('센서 조회 실패:', type(error).__name__)
    time.sleep(1)
PY
```

`connected=true`만으로 BU04 측정이 유효하다고 판단하지 않는다. 가상 추종에는 유효 표본·태그 ID 일치·유일한 유효 추적 대상이 필요하다. 이 조건이 없으면 토글을 켜도 가상 주행은 정지한다. LiDAR 전체 스캔에 무효 빔이 남아도 현재 가상 입력 검증이 정지시킬 수 있다. 검증을 통과시키려고 무효 상태를 참으로 바꾸거나 센서 안전 제한을 낮추지 않는다.

## 새 Pi 실행 후보의 가상 조종

PC의 `pi-runtime/`은 아직 Pi에 배포되거나 Ubuntu에서 빌드된 적이 없다. 이 절은 배포 후 경로와 설정을 확인하기 위한 개발 절차다. 이전 18084 화면은 아래와 별도의 프로그램이다. 실제 장치 설정과 HTTPS 인증서는 각 Pi에 맞게 별도로 확인하며, 원본 ZIP에 포함된 개인키나 검수되지 않은 보정값을 다른 장치에 그대로 적용하지 않는다.

Ubuntu 22.04 ARM64에서 ROS 2 Humble·colcon·gcc·python3·aiohttp 및 필요한 ROS 패키지를 준비하고, **배포한 `pi-runtime/` 폴더로 이동한 뒤** 실행한다.

```bash
bash prepare_ros.sh
```

빌드 전후 `bash check_pi.sh`는 장치를 열지 않고 환경·빌드 파일 존재 여부를 출력한다. 실제 빌드는 Ubuntu 22.04 ARM64·ROS 2 Humble에서 실행해야 한다. 스크립트는 현재 위치의 `.runtime/`에 ROS 작업공간을 만들고 BU04 C 파서를 빌드한다. 장치 설정은 포함되어 있지 않으므로 기존 백업을 보존한 상태에서 해당 Pi의 설정·USB 장치 경로를 확인해야 한다. 폴더 위치가 바뀌면 다시 빌드한다. 센싱을 시작할 준비가 되면 같은 폴더에서 다음을 실행한다.

```bash
bash sensors.sh
```

센싱이 이미 실행 중이면 중복 실행하지 않고 같은 `pi-runtime/` 폴더에서 `bash check_pi.sh --sensors`로 8765의 LiDAR·BU04·추적 상태를 5회 조회한다. 가상 감시 기준과 센서 연결 확인이며 실제 위치 정확도나 구동 승인은 아니다.

별도 Pi 터미널에서 같은 `pi-runtime/` 폴더로 이동해 검증된 인증서 폴더와 실제 Pi IP를 지정한다.

```bash
export SMARTCART_CERT_DIR="$HOME/smartcart-radar-https-check-PyDCFCjn/cert"
bash manual.sh 192.168.43.212
```

이 명령은 **가상 조종**을 HTTPS 8081에서 연다. 화면 접속키와 별도 조이스틱 키를 사용한다. 실제 GPIO 서버가 같은 포트를 사용 중이면 중복 시작하지 않는다. LiDAR 또는 BU04 무효·ID 불일치·미관측 빔·장애물·시간 초과 시 가상 출력은 잠긴다. 센서 조회 병렬화 이후 합성 HTTP·WebSocket 시험은 6회 통과했으나 실제 Pi 통신과 실측은 추가 확인해야 한다.

## 이전 실측 센서 기반 가상 자율주행 화면

이 절은 기존 18084 시험 묶음의 절차다. 새 `pi-runtime/`의 가상 조종 서버와 다른 버전이다. 실제 카트 이동이나 GPIO 출력이 아닌, 실측 센서 데이터에 대한 브라우저 미리보기다. 화면 접속키와 조이스틱 연결키는 서로 다르다. 화면 접속키는 숫자 4자리이며 같은 사설망의 시험용으로 제한한다.

2026-11-19 레이아웃·진단 후보는 토글을 헤더 중앙에 놓고 가상 설정을 접는다. 사용자가 Pi Chrome에서 방향 버튼 해제 후 활성화가 꺼진다고 보고했으나 원인은 미확인이다. 조이스틱 옆의 정지 사유를 확인하며 이 후보로 해결되었다고 판단하지 않는다. 브라우저별 화면 적합성과 Pi 적용 결과는 미검증이다.

2026-11-19 화면 수정본은 UWB 마름모만 레이더 화면 아래로 1.25m 이동한다. 측정 좌표·추적 판정·가상 주행 계산에는 적용하지 않으며 한 지점 관찰에 따른 임시 표시 보정으로 정확도가 검증되지 않았다. 자율주행·장애물 회피·사용자 추종 토글은 상단에 있다. 실측 입력이 무효·만료되면 자율주행은 OFF로 유지하고 상단에 사유를 표시한다. 복구 후 사용자가 다시 켜야 한다. 이 묶음과 수정은 기존 Pi 설치본과 새 `pi-runtime/`에 자동 반영되지 않는다.

### GitHub 소스로 실행하기

저장소에 코드가 업로드된 뒤 Pi에서 별도 디렉터리로 클론한다. 기존 Pi 설치 폴더와 센싱 서버를 덮어쓰거나 종료하지 않는다. 네트워크 연결과 Python `aiohttp`, Pi가 신뢰하는 HTTPS 인증서가 필요하다.

```bash
cd "$HOME"
git clone https://github.com/Protocol314/smartcart.git smartcart-source
```

이미 `~/smartcart-source`가 있으면 그 폴더를 덮어쓰지 말고 다른 새 폴더 이름을 사용한다. 포트 18084와 인증서 확인은 아래 절과 동일하다. 확인 후 다음 명령으로 가상 화면만 실행한다.

```bash
task_cert="$HOME/smartcart/smartcart-virtual-lan-cert"
python3 -u "$HOME/smartcart-source/pc-development/virtual-lab-18084/virtual_lan_server.py" \
  --address 192.168.43.212 --port 18084 \
  --cert "$task_cert/server.crt" --key "$task_cert/server.key"
```

이 소스 실행 방식은 Pi 적용과 실물 브라우저 시험이 아직 확인되지 않았다. `pi-runtime/pi-export-reference/`의 실제 GPIO 원본을 자동 실행하지 않는다.

### 로컬 시험 묶음 전송

이 ZIP은 로컬 `build/releases/`에 보관하며 GitHub 소스 저장소에는 포함하지 않는다. PC에서 프로젝트 루트로 이동한 후 PowerShell에서 실행한다.

```powershell
$taskZip = Join-Path (Get-Location) 'build\releases\smartcart-virtual-layout-diagnostic-20261119.zip'
Get-FileHash -Algorithm SHA256 -LiteralPath $taskZip
scp "$taskZip" capstone@192.168.43.212:~/smartcart-virtual-layout-diagnostic-20261119.zip
```

기대 SHA-256:

```text
09a578955d6b7d49fd8a9817697073b5ae24425592e9cdbd472de142e4a7492a
```

Pi에 접속한 별도 SSH 창에서 파일 해시와 HTTPS 인증서를 확인한다.

```bash
sha256sum "$HOME/smartcart-virtual-layout-diagnostic-20261119.zip"
task_cert="$HOME/smartcart/smartcart-virtual-lan-cert"
openssl verify -CAfile "$task_cert/ca.crt" -verify_ip 192.168.43.212 "$task_cert/server.crt"
```

해시가 일치하고 인증서 검증이 `server.crt: OK`일 때만 다음 단계로 진행한다. 다른 Pi 주소를 쓰면 인증서도 해당 IP를 포함해야 한다. 인증서 오류 경고를 우회하지 않는다.

### 서버 실행

먼저 이전 가상 서버 창이 있으면 그 창에서 `Ctrl+C`로 정상 종료한다. 포트 18084가 이미 사용 중이면 다른 포트를 택하기 전에 아래 읽기 전용 확인을 실행한다. 모르는 프로세스를 강제 종료하지 않는다.

```bash
ss -ltnp '( sport = :18084 )'
```

새 Pi SSH 창에서 별도 임시 폴더에 압축을 푼다.

```bash
task_cert="$HOME/smartcart/smartcart-virtual-lan-cert"
task_dir="$(mktemp -d "$HOME/smartcart-virtual-4digit.XXXXXXXX")"
python3 -m zipfile -e "$HOME/smartcart-virtual-layout-diagnostic-20261119.zip" "$task_dir"
```

같은 창에서 지정한 Pi 주소로 서버를 시작한다.

```bash
python3 -u "$task_dir/virtual_lan_server.py" \
  --address 192.168.43.212 \
  --port 18084 \
  --cert "$task_cert/server.crt" \
  --key "$task_cert/server.key"
```

접속키를 숫자 4자리로 정해 두 번 입력한다. 입력 중 문자가 보이지 않는 것은 정상이다. 잘못된 키를 5번 입력하면 15분 동안 로그인이 제한된다. 서버 터미널에는 별도의 가상 조이스틱 연결키가 표시된다. 서버가 시작되면 창을 유지한다.

### 브라우저 시험

PC·태블릿·휴대폰 브라우저에서 다음 주소를 연다.

```text
https://192.168.43.212:18084/
```

화면 접속키로 로그인한다. 하단 **센서 및 실시간 파라미터**에서 `SOURCE`를 **Pi 센싱 서버**로 두고 LiDAR 갱신, BU04 측정 유효 및 ID 일치, 추적 상태를 확인한다. 가상 입력을 **실측 LiDAR + BU04**로 선택하고 사용자 추종·장애물 회피·자율주행 토글을 켠다. 가상 속도·회전과 상태 문구만 시험한다. 가상 카트 위치의 누적은 실제 차체 움직임을 뜻하지 않는다.

무효·만료된 LiDAR, BU04 무효 또는 ID 불일치, 추적 후보 모호, 화면 포커스 손실·STOP 조건이면 가상 실행은 정지한다. 정지 원인을 확인하고 센서가 복구된 뒤 다시 켠다. Pi의 실제 모터 출력은 이 서버에서 사용할 수 없다.

시험 완료 후 가상 서버 창에서 `Ctrl+C`로 종료한다. 센싱 창은 별도로 유지하거나, 직접 시작한 경우에만 해당 창에서 종료한다.

## 실제 GPIO 서버의 잠금 오류 확인

실제 GPIO 서버는 가상 시험 서버와 별도이며, LiDAR 장애물 회피·BU04 추종 제어를 제공하지 않는다. 실제 주행 자격이 검수되지 않은 상태에서 주행하지 않는다. 전진 극성, 독립 물리 차단과 Pi 피드백, UWB 거리 모델, LiDAR 장착 및 정지영역, 저속 정지·재시작 조건을 먼저 실물에서 확인해야 한다. 기존 사용자 Pi에서 4자리 GPIO 변경이 실제 적용됐다는 증거는 없다.

GPIO 서버 실행기 잠금 오류가 발생하면 먼저 소유 프로세스와 잠금 사용자를 확인한다. 잠금 파일을 삭제하지 말고 이미 살아 있는 센싱 서버를 중복 실행하지 않는다.

```bash
ss -ltnp '( sport = :8081 or sport = :8765 )'
fuser -v /tmp/smartcart-runner-*/live.lock /tmp/smartcart-runner-*/8765.lock /tmp/smartcart-runner-*/motor.lock
ps -ww -eo pid,ppid,args | grep -E '[d]rive.py|[c]art.py|[s]martcart_runner|[s]ensing.py'
```

GPIO 서버를 사용할 때는 사용자가 설치한 실제 코드 버전과 HTTPS 인증서 IP를 먼저 대조한다. 이전 실제 수동 서버 절차는 포트 8081이고 가상 시험 서버는 18084다. 둘을 같은 포트로 동시에 시작하지 않는다. 2초 내 PWM 응답 확인 실패 시 정지 요청 및 재활성화 동작은 보존해야 하며, Pi GPIO 설치본에 실제 적용됐는지 별도로 검증한다. 엔코더가 없으므로 PWM 상태 확인은 실제 바퀴 움직임 확인이 아니다.

## 종료 및 장애 확인

- 직접 시작한 서버만 해당 터미널에서 `Ctrl+C`로 정상 종료한다.
- `18084`가 사용 중이면 `ss`로 프로세스를 확인한다. 정체가 확인된 이전 가상 서버만 그 서버의 터미널에서 종료한다.
- `8765`가 열려 있고 `/api/health`가 200이며 센싱 프로세스가 있으면 센싱을 다시 시작하지 않는다.
- 잠금 파일 제거, 확인되지 않은 프로세스 강제 종료, 무효 센서 상태 우회는 하지 않는다.
- TLS 인증서 오류는 무시하지 말고 인증서 주소·유효기간·단말의 CA 신뢰를 확인한다.

센싱 종료가 필요할 때만 센싱 실행 터미널에서 `Ctrl+C`를 누른다. 실제 GPIO 경로는 위의 안전 조건과 적용 버전이 확인될 때까지 실행하지 않는다.
