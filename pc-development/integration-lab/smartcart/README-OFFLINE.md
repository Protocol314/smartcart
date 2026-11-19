# Pi·센서 없는 합성 재생과 사전 준비

2026-10-26. 통합 실행기는 Python 3를 사용하며 npm 설치가 필요 없다.
기본 모드는 합성 재생이고 실제 장치 실행은 `--mode live`를 명시해야 한다.
모터 출력·AT 명령·플래시·펌웨어 변경은 제공하지 않는다.

2026-10-27 현재 Pi 실측 설치·수신·웹 화면 확인은 완료했다. 실제 운영은
[통합 README](README.md)의 `start-live.sh`, 아래 내용은 센서 없는 재생·사전 준비에 사용한다.
현재 설치 상태와 검수 한계는 [기술 백서](../docs/SMARTCART_WHITEPAPER.md)에 있다.

## Windows 개발 저장소

저장소 루트 PowerShell에서:

```powershell
.\smartcart\scripts\start-cart.bat
```

브라우저에서 `http://127.0.0.1:8765`를 연다. 종료는 Ctrl+C다.
Python 3와 PC용 C 파서가 필요하다. 라이브러리가 없으면
`python smartcart/scripts/build_uwb_native.py --cc 확인한_GCC_경로`로 명시적으로 빌드한다.
Pi 배포 ZIP의 ARM64 `.so`는 Windows에서 실행하지 않는다.

## Linux·Pi 배포 폴더

Pi 4 Ubuntu 22.04 ARM64 배포본은 ARM64 파서를 포함한다.
설치 후 `~/SmartCart-D5-v2/smartcart`에서:

```bash
python3 cart.py check
bash scripts/start-cart.sh
```

재생에는 ROS·센서가 필요 없다. Windows 개발 저장소는 같은 명령에서 `python3`를 `python`으로 바꾼다.
GUI의 REPLAY·합성 표시와 구동 불가 표시를 확인한다. 웹 서버 준비는 센서 검수 완료를 뜻하지 않는다.

## 설정 준비와 자동 검사

`smartcart` 디렉터리에서 실행한다. 기존 디렉터리는 덮어쓰지 않는다.

```bash
python3 cart.py init --directory local-config
python3 cart.py --config local-config/run.json plan --mode live
python3 scripts/verify-offline.py
```

`plan`은 장치를 열거나 프로세스를 시작하지 않는다. 새 설정은 USB 포트와 식별값이
미배정이고 장착·거리 가정이 미검수라 준비 부족(종료 코드 3)이 정상이다.
지금은 `verified`를 true로 바꾸지 않는다. 실제 장치를 하나씩 대조한 뒤 확정한다.
`run.json`의 상대 경로는 해당 파일이 있는 디렉터리를 기준으로 해석한다.

검증기는 C ABI·합성 고장/복구·추적 기대값·실제 HTTP 재생·종료 후 위치 제거와 포트 해제를 검사한다.
`--output 새_보고서.json`을 지정할 때만 좌표·태그·원문 USB 식별자 없는 로컬 집계를 저장한다.
기본 파일 기록·외부 전송은 없다. 실패나 시간 초과를 통과로 표시하지 않는다.

## 실제 연결 후 명령 하나로 시작

최신 소스의 `scripts/start-live.sh`는 ROS Humble, 기존 LiDAR workspace와 통합 workspace 환경을 직접 불러온다. 실물 USB 배정과 `local-config/run.json` 준비를 한 번 완료하면 이후 새 SSH 창에서도 다음 한 명령으로 실측을 시작한다.

```bash
bash "$HOME/SmartCart-D5-v2/smartcart/scripts/start-live.sh"
```

이 스크립트는 기존 0.3.1 ZIP에 포함되지 않은 추가 파일이며 현재 Pi에는 반영돼 있다. 새 설치는 런북 6.3절의 후속 파일까지 반영한다. 인수 없는 기존 `start-cart.sh`는 계속 합성 재생을 시작한다. `start-live.sh`는 기존 실행기의 USB 식별 검사와 중복 실행 방지를 거쳐 LiDAR 드라이버·좌우 UWB 수집·추적 화면을 함께 시작한다. 종료는 같은 창에서 Ctrl+C다.

추가 도구 `scripts/prepare-live.py`는 `--left-serial`, `--right-serial`, `--lidar-serial`, `--tag-address`로 지정한 현재 장치를 VID/PID·시리얼·독립 물리 장치 여부로 대조한다. `--confirmed-physical-labels`는 물리 분리 시험 등으로 역할을 확인한 경우에만 지정한다. 기존 로컬 설정을 백업하고 USB 배정과 태그 주소를 반영하며, 실측하지 않은 장착·거리 검수 플래그는 유지한다. AT 명령과 포트 개방은 없다.

모터 드라이버가 조립돼 있어도 현재 패키지는 모터 제어 명령을 출력하지 않는다. `/scan` 및 `/smartcart/tracking/target`, `/smartcart/tracking/status`는 이후 제어기가 사용할 센싱 경로다. 실제 제어 인터페이스와 정지·타임아웃 동작은 연결된 드라이버에 맞춰 구현·검증해야 한다.

기존 ROS Humble·LiDAR 워크스페이스와 새 센싱 workspace의 `install/setup.bash`를 source한다.
`field_audit.py inventory --show-identifiers`로 세 장치의 포트를 실제 라벨과 대조하고
로컬 설정에 정확히 기록한다. 기본 예제의 장착·거리 가정은 실측 없이 확정하지 않는다.

```bash
python3 cart.py --config local-config/run.json check --mode live
python3 cart.py --config local-config/run.json start --mode live
```

`lidar_mode=managed`는 확인한 C1 드라이버·좌우 UWB 수집·추적 GUI를 시작한다.
이미 드라이버가 별도로 실행 중이면 `lidar_mode=external`을 사용한다.
중복 실행·기동 실패·자식 프로세스 종료는 전체 실행을 중단하며 외부 드라이버는 종료하지 않는다.
실측은 HTTP 포트가 달라도 사용자당 한 번만 허용한다. 재생은 포트별로 독립 실행할 수 있다.
`--seconds 10`처럼 시간을 명시하면 서버 기동 후 해당 시간 동안 실행하고 종료한다.

HTTP는 `127.0.0.1`에만 바인딩하며 PC에서 보려면 SSH 터널을 사용한다.
ROS도 같은 Pi localhost를 사용한다. 기존 ROS 도메인 번호는 유지한다.
데이터가 없거나 무효·만료되면 위치를 숨긴다. 모든 구동 허가는 false다.

실물 검수 절차는 배포 루트의 `README-PI4.md`를 따른다.
Pi 실기기·USB 역할 배정·LiDAR와 좌우 BU04의 동시 수신·웹 표시는 확인했다.
동시 UWB 무효 원인·거리 정확도·장착·장시간 전원 안정성·사람 추종과 모터 검수는 남아 있다.
