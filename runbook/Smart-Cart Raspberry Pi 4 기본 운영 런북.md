# SmartCart Raspberry Pi 4 시연 운영 런북

Pi 4 · Ubuntu 22.04 ARM64 · ROS 2 Humble · C1 LiDAR · BU04 · BU03 태그

## 실행 전 확인

실제 카트의 기본 순서는 **SSH 접속 → LiDAR·BU04 통합 센싱 → 실제 조종 서버**다. 아래 센싱은 사용자가 Pi에서 실행했다고 보고한 단일 BU04 구성이다. 현재 Pi IP·장치 연결과 코드 경로를 다시 확인한다.

**실제 조종 서버는 현장 안전 검수와 적용 버전 확인 후에만 시작한다.** 기존 Pi 설치본에 LiDAR·UWB 이상 잠금과 2초 PWM 응답 감시가 적용됐다는 증거는 아직 없다. 제공된 기존 수동 코드의 상태에는 `sensor_interlock=false`가 표시된다. 이 확인이 끝나지 않았다면 3단계 서버 실행은 보류한다. 센서 무효·ID 불일치·시간 초과 잠금과 비상정지를 우회하지 않는다.

2026-11-24 사용자가 Pi에서 확인한 `follow_config.json`의 `age_limit_s`와 `pwm_cap`은 GitHub 검토본의 상한(각각 0.20초, 30)보다 컸다. `lidar_enabled`는 재검사에서 `true`였으나 앞선 결과와 달랐다. 서버 시작 시 읽힌 값과 센서 이상 시 실제 정지·재시작 조건은 확인되지 않았다. 설정이나 AT 백업을 임의로 바꾸지 않는다.

전진 극성, 독립 모터 전원 차단, 정지 및 재시작 조건은 실물 확인이 필요하다. 비상정지는 모터 전원만 차단하며 Pi GPIO 피드백이 없고, 엔코더도 없다. PWM 표시나 센서 HTTP 응답은 실제 바퀴 움직임·안전성의 증거가 아니다. BU03/BU04 설정을 바꾸기 전에는 AT 설정 백업을 보존한다.

## 1. SSH 접속

PC PowerShell에서 실행한다. 아래 주소는 이전에 확인한 예시이며, 현재 Pi 주소는 Pi에서 `hostname -I`로 확인한다.

```powershell
ssh capstone@192.168.43.212
```

## 2. LiDAR·UWB 통합 센싱

LiDAR와 BU04를 Pi에 연결하고 BU03 전원을 켠 뒤, Pi 터미널에서 기존 센싱을 확인한다.

```bash
ss -ltnp '( sport = :8765 )'
curl -sS --max-time 2 -o /dev/null -w '센싱 HTTP 상태: %{http_code}\n' http://127.0.0.1:8765/api/health
ps -ww -eo pid,ppid,args | grep -E '[c]art.py|[s]martcart_runner|[s]ensing.py'
```

HTTP 200과 실행 중인 센싱 프로세스가 있으면 그대로 사용하고 다시 시작하지 않는다. 응답이 없을 때는 잠금 보유자를 확인한다.

```bash
fuser -v /tmp/smartcart-runner-*/live.lock /tmp/smartcart-runner-*/8765.lock
```

8765 서버와 잠금 보유자가 모두 없을 때만 아래 명령을 **한 번** 실행하고 터미널을 켜 둔다. 이 명령 하나가 LiDAR와 왼쪽 BU04의 통합 센싱을 시작한다. 별도 UWB 수집기를 중복 실행하지 않는다.

```bash
bash "$HOME/smartcart/SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh"
```

이 명령은 모터를 구동하지 않는다. 단일 BU04 수평거리 가정·장착·위치 정확도는 검수 전이다. 장치 연결이나 HTTP 200만으로 측정이 유효하다고 판단하지 않는다.

### 실시간 센서 상태 확인

별도 Pi SSH 터미널에서 아래 명령을 실행한다. 위치와 태그 식별번호는 출력하지 않는다.

```bash
python3 - <<'PY'
import json
import math
import time
from urllib.request import urlopen

def read(path):
    with urlopen('http://127.0.0.1:8765' + path, timeout=2) as response:
        return json.load(response)

for _ in range(3):
    try:
        frame = read('/api/frame')
        scan = frame.get('scan') or {}
        values = scan.get('ranges') or []
        low, high = scan.get('range_min'), scan.get('range_max')
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (low, high)):
            raise ValueError('LiDAR 거리 범위 오류')
        invalid = sum(type(v) not in (int, float) or not math.isfinite(v) or v < low or v > high for v in values)
        print('LiDAR:', frame.get('state'), '출처:', frame.get('source_mode'),
              '나이 ms:', frame.get('age_ms'), '전체:', len(values), '무효:', invalid)
        uwb = read('/api/dual-uwb')
        left = (uwb.get('sensors') or {}).get('uwb_left') or {}
        print('BU04:', left.get('state'), '연결:', left.get('connected'),
              '측정 유효:', left.get('measurement_valid'), 'ID 일치:', left.get('id_valid'),
              '사유:', left.get('invalid_reason'))
        track = (read('/api/tracking').get('result') or {}).get('track') or {}
        print('추적:', track.get('state'), '유효:', track.get('valid'),
              '후보:', track.get('candidate_count'), '사유:', track.get('invalid_reason'))
    except Exception as error:
        print('센서 확인 실패:', type(error).__name__)
    time.sleep(1)
PY
```

LiDAR의 지속적인 갱신과 무효 빔, BU04 측정 유효·ID 일치, 단일 유효 추적 대상을 확인한다. 센서 오류가 있으면 원인을 해결하고 구동을 보류한다. 과거에는 무효 LiDAR 빔과 BU04 무효 측정이 보고됐으며 현재 상태는 재확인해야 한다.

## 3. 실제 조종 서버

기존 Pi 서버의 설치 경로와 적용 버전을 확인한다. 2026-11-24에는 사용자가 `~/smartcart/releases/real-uwb-v1/server.py`의 GPIO 실행을 확인했고, 나중에 확인한 조종 포트는 수신 중이 아니었다. 이는 당시 관측이며 현재 상태나 무장·센서 잠금 상태를 뜻하지 않는다. 제공받은 이전 원본에는 `~/smartcart/manual.sh`를 사용하는 별도 수동 서버도 있다. 다음은 읽기 전용 확인이다.

```bash
ss -ltnp '( sport = :8080 or sport = :8081 or sport = :18084 )'
ps -ww -eo pid,ppid,args | grep -E '[r]eal-uwb-v1/server.py|[d]rive.py|[m]anual.sh'
sha256sum "$HOME/smartcart/manual.sh" \
  "$HOME/smartcart/SmartCart_ManualDrive_180/smartcart/drive.py" \
  "$HOME/smartcart/SmartCart_ManualDrive_180/smartcart/smartcart_motor/manual_remote.py" \
  "$HOME/smartcart/SmartCart_ManualDrive_180/smartcart/web/js/manual_controls.js"
```

조종 포트가 사용 중이면 어떤 서버인지 확인하고 중복 실행하지 않는다. 프로세스 목록이나 터미널 출력에 접속키가 포함된 경우 공유하지 않는다. 해시는 버전 비교용이며 일치만으로 안전 검수를 대신하지 않는다. 제공받은 이전 수동 원본 `manual_remote.py`의 SHA-256은 `06a0a6f9a1da04d37dd895f65135850262fc1752fecb7303f85092e244f1c276`이며, 그 원본 자체에는 센서 자동 잠금이 연결돼 있지 않다. 이를 검증된 통합 서버로 취급하지 않는다.

기존 Pi 실제 서버의 실행 형식은 다음과 같다. **위 안전 조건, 실제 설치 경로·버전 및 센서 이상 잠금·2초 응답 감시가 확인된 경우에만** 별도 Pi SSH 터미널에서 실행한다. 현재는 이 조건이 미확인이므로 시연 구동을 보류한다.

```bash
bash "$HOME/smartcart/manual.sh" 192.168.43.212
```

서버가 시작되면 출력된 HTTPS 주소와 연결키를 해당 실행 터미널에서 확인하고, 같은 사설망의 브라우저에서 접속한다. 이전 주소 예시는 `https://192.168.43.212:8081/`이다. TLS 인증서는 해당 IP와 단말의 CA 신뢰를 확인하며 경고를 우회하지 않는다. 접속키나 개인키를 채팅에 공유하지 않는다.

이 기존 서버에서 LiDAR·BU04가 화면에 보이는 것과 실제 자동 추종·장애물 회피가 동작하는 것은 다르다. Pi에서 추출한 UWB 추종·LiDAR 처리 코드는 GitHub `main`의 `pi-runtime/pi-field-snapshot/`에 병합됐다. 이 경로는 안전 검토본으로 실제 GPIO를 잠그며, 장치 설정도 포함하지 않으므로 Pi의 운행용 설치본이 아니다. GitHub 병합은 기존 Pi 코드를 변경하지 않는다.

추출 당시 `real.sh`와 `virtual.sh`는 모두 기본 GPIO를 선택하도록 돼 있었다. 파일 이름만으로 가상 출력이라고 판단하지 말고 Pi의 실제 파일·서버 상태를 확인한다. 시험용 합성 설정을 실제 설정으로 복사하거나 안전 잠금을 해제하지 않는다.

## 종료와 오류

- 종료할 때는 직접 시작한 실제 조종 서버 터미널에서 `Ctrl+C`를 누르고, 센싱도 직접 시작했다면 해당 센싱 터미널에서 별도로 종료한다.
- `8765`·`8081` 포트나 실행기 잠금이 사용 중이면 소유 프로세스를 확인한다. 알 수 없는 프로세스를 강제 종료하거나 잠금 파일을 지우지 않는다.
- 센서 이상, 2초 응답 미확인, 비상정지 이상 또는 검수 불일치 시 구동을 중단하고 모터 전원을 차단한다.
- 상세 작업 이력과 미검증 사항은 `HANDOFF.md`, 장치·모터 검수 사항은 `docs/`를 따른다.
