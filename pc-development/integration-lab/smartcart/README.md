## 양쪽 모터 제어 개발 모듈

PC 작업공간의 `smartcart_motor`에는 전후·제자리 회전 계산, 듀티 조절과 안전 잠금이 추가되었다. 사용자 설명상 모터 출력선 극성은 좌우가 반대로 연결되어 같은 방향 채널로 직진한다. RPWM·LPWM 중 실제 전진 채널의 최종 지정은 기록되지 않았고 실측 안전 상태 공급자와 통합 실행기 연결은 없다. 사용자가 비상정지는 모터 전원만 차단하며 Pi GPIO 피드백이 없다고 보고했으므로 안전 상태 잠금을 유지한다. 기본 상태에서는 GPIO 구동을 허용하지 않는다. 엔코더가 없어 PWM은 실측 속도가 아니다. [상세 및 가상 검사](../docs/SMARTCART_MOTOR_CONTROL.md).

```bash
python -m smartcart_motor
python -m unittest discover -s tests -p test_motor_control.py
```

이 명령들은 PC 작업공간에서 가상 검사를 수행한다. 기존 Pi 설치본에 모듈이 있다고 가정하지 않는다.

# SmartCart D5-v2 통합 센싱

기준: 2026-10-27. Raspberry Pi 4 / Ubuntu 22.04.5 ARM64 / ROS 2 Humble에서
RPLIDAR C1·좌우 BU04·BU03 태그를 사용한다. LiDAR 전용 v1.0.1의 복사본을 확장한
현재 통합 프로그램이며, Pi 배포는 0.3.1 + 후속 업데이트, 센싱 런타임 표시는 0.1.0이다.

## 평시 실측 시작

현재 Pi는 설치·빌드·장치 배정·후속 업데이트가 완료돼 있다. SSH 창에서:

```bash
bash "$HOME/SmartCart-D5-v2/smartcart/scripts/start-live.sh"
```

스크립트가 ROS와 workspace 환경을 source하고, 기존 `local-config/run.json`을 검사한 뒤
관리 모드에서 C1 드라이버·좌우 UWB 수집기·추적/웹 서버를 함께 시작한다.
별도 UWB 수집기나 LiDAR 드라이버를 중복 실행하지 않는다. 종료는 같은 창의 Ctrl+C다.
모터 출력·AT 설정·펌웨어 플래시는 이 실행에 포함되지 않는다.

PC의 별도 PowerShell에서 SSH 터널을 유지한다. Pi 주소는 현재 네트워크에서 확인한다.

```powershell
$PiAddress = Read-Host '현재 Pi IP 또는 확인된 호스트명'
ssh -N -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 "capstone@$PiAddress"
```

브라우저: `http://127.0.0.1:8765/`. Pi 자체 브라우저에서도 같은 주소를 사용한다.
기존 터널이 있으면 새 터널은 만들지 않는다. 웹은 Pi 루프백에 바인딩한다.

## 무엇이 실행되는가

```text
BU03 태그(별도 전원) ── UWB ── 왼쪽 BU04 데이터 USB ─┐
                          └─ 오른쪽 BU04 데이터 USB ─┼─ Pi 수집·ROS·추적/웹
C1 LiDAR ── USB ── sllidar_ros2 ── /scan ────────────┘
PC 브라우저 ── SSH 터널 ── Pi 127.0.0.1:8765
```

| 경로 | 기능 |
|---|---|
| `scripts/start-live.sh` → `cart.py ... start --mode live` | ROS 환경·설정·USB 식별 검사와 프로세스 감독 |
| `sllidar_ros2` | C1 460800 baud, `/scan`, `lidar_scan` 프레임 |
| `smartcart_uwb` | 독립 BU04 두 USB, 115200/8N1, 기존 C 파서와 관측·전체 상태 |
| `smartcart_tracking` | UWB 위치 추정·LiDAR 군집·대상 연관, 미검증 결과는 숨김 |
| `smartcart_sensing` + `web/` | LiDAR·좌우 원시 상태·추적 표시, 읽기 전용 HTTP/SSE |

현재 센싱 경로에는 ESP32가 필요하지 않다. ESP32와 모터 드라이버의 제어 인터페이스는 후속 작업이다.

## 화면과 수신 확인

- LiDAR: 약 10Hz·720 rays의 실제 수신을 확인했다. 화면 배경은 1/2Hz로 갱신한다.
- 좌우 BU04: 거리(m), 원시 각도(deg), 측정 유효·태그 ID 일치, 수신 경과(ms)를 최대 10Hz 표시한다.
- 무효 값은 `범위 밖`·`측정 무효`, 단절·만료는 별도 상태로 표시하며 과거 값을 숨긴다.
- 원시 각도는 BU04 오른쪽이 양수다. ROS `base_link`는 +X 전방 / +Y 좌측이다.
- PC 작업공간에는 좌우 가정 평균과 LiDAR 인접 군집을 비교하는 읽기 전용 실험 화면이 준비돼 있다. Pi에 별도 설치기를 적용해야 하며, 결과는 추적·구동 승인이나 사람 판별이 아니다. [설명과 적용 범위](../docs/SMARTCART_FIELD_PREVIEW.md).
- 보기 반경 3m는 화면 자르기이며 C1 측정 설정이나 UWB 유효 범위를 바꾸지 않는다.
- `uwb.state=off`는 기존 ESP32 입력의 상태다. 현재 좌우 상태는 `dual_uwb`와
  `sensing.uwb_backend=dual`을 확인한다.

프로그램이 실행 중인 Pi의 다른 터미널에서 현재 값만 조회할 수 있다.

```bash
python3 - <<'PY'
import json
from urllib.request import urlopen
for endpoint in ('health', 'dual-uwb'):
    with urlopen('http://127.0.0.1:8765/api/' + endpoint, timeout=3) as response:
        print(endpoint, json.dumps(json.load(response), ensure_ascii=False, indent=2))
PY
```

조회는 USB를 다시 열지 않으며 파일을 남기지 않는다. `health`는 상태 집계,
`dual-uwb`는 현재 원시 표시 값이다. 웹 서버 기동 로그의 `sensor_freshness_verified=false`는
기동만으로 센서 검수를 완료하지 않는다는 뜻이다. 실제 수신 상태는 위 조회와 화면으로 확인한다.

## 실행 명령 구분

| 명령 | 결과 |
|---|---|
| `bash ~/SmartCart-D5-v2/smartcart/scripts/start-live.sh` | 현재 Pi의 배정된 실물 센서·GUI 시작. 인수 없음 |
| `bash scripts/start-cart.sh` / Windows `scripts\start-cart.bat` | 합성 통합 재생. 실물 시작 아님 |
| `python3 cart.py --config local-config/run.json check --mode live` | ROS 환경을 미리 source한 뒤 설정·환경·USB 열거만 검사 |
| `python3 cart.py --config local-config/run.json start --mode live` | 위 환경에서 통합 실행기를 직접 호출 |
| `python3 radar.py serve --mode ros` | LiDAR 전용 진단. 드라이버·UWB를 함께 시작하지 않음 |

`npm start`는 사용하지 않는다. 인수 없는 `start-cart`를 실측 시작으로 바꾸지 않는다.
`start-live.sh`는 기본 `workspace/install/setup.bash`를 사용하므로 다른 workspace 경로는
환경을 직접 source한 후 `cart.py`로 실행한다.

## 설치 상태와 남은 범위

현재 Pi에는 0.3.1 설치·ROS 빌드 후 USB 링크 열거 수정, `start-live.sh`·`prepare-live.py`,
좌우 화면·TCP 재시작 업데이트가 적용돼 있다. 기존 0.3.1 ZIP은 그대로 보존했으며
새 장비 설치는 [Pi 배포 안내](README-PI4.md)와 런북의 후속 파일 반영 단계까지 진행한다.
이미 운영 중인 Pi에는 초기화·장치 재배정·빌드를 반복할 필요가 없다.

동시 UWB에서 간헐적인 무효 측정이 남아 있다. 거리 모델과 장착이 미검증이면
원시 수신이 정상이어도 사람 위치·대상 추적은 유효하지 않을 수 있다.
사용자는 MY1016Z·BTS7960 IBT-2와 Pi 사이 전선·공통 GND·비상정지 장치를 연결했고 바퀴는 미장착이라고 밝혔다. 제시한 수동 시험 코드는 [motor_bench_manual.py](scripts/motor_bench_manual.py)에 보존하고 Pi 전송 파일의 해시와 RPi.GPIO import를 확인했다. 사용자는 실물 실행 후 모터가 잘 돌아간다고 보고했다. 각 모터의 실제 방향·정지와 비상정지의 전원 차단 결과는 별도 확인이 필요하다.
이 코드는 실행 즉시 양쪽 모터를 전진·후진시키는 독립 시험이며 센싱 한 명령과 연결되지 않는다. 센서 기반 제어·정지 연동은 미구현이고 `drive_authorized=false`다. 화면 정지는 하드웨어 정지가 아니다.

## 문서

- [기술 백서](../docs/SMARTCART_WHITEPAPER.md): 설계·구현·데이터 계약·현재 검증 범위
- [통합 센싱 안내](README-SENSING.md) · [Pi 설치](README-PI4.md) · [합성 재생](README-OFFLINE.md)
- [운영 런북 MD](<../runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.md>) · [HTML](<../runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.html>)
- [좌우 화면과 업데이트](../docs/SMARTCART_DUAL_UWB_UI.md) · [검증 기록](../docs/SMARTCART_VALIDATION.md)
- [ROS·추적 런타임](../docs/SMARTCART_TRACKING_RUNTIME.md) · [변경 내역](CHANGELOG.md)
