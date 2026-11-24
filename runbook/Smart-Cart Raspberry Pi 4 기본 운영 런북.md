# SmartCart Raspberry Pi 4 운영 런북

Pi 4 · Ubuntu 22.04 ARM64 · ROS 2 Humble · C1 LiDAR · 단일 BU04 · BU03 태그

## 실제 카트의 현재 상태

Pi에서 사용 중인 기존 실제 수동 조종 코드와 아래 가상 서버는 별개다. 제공받은 기존 코드가 실제로 동작했다는 사용자 보고는 있으나, 현재 Pi 설치본의 해시·전진 극성·정지 및 재시작 조건은 확인되지 않았다. 기존 수동 서버에 LiDAR·UWB 이상 잠금과 2초 PWM 응답 감시가 적용됐다는 증거도 없다. 엔코더가 없어 PWM 응답은 바퀴 움직임의 실측이 아니다.

실제 GPIO 구동은 검증된 현장 절차와 적용 버전을 확인한 뒤에만 진행한다. 모터 전원 차단용 비상정지에는 Pi 피드백이 없다. 센서 무효·ID 불일치·시간 초과와 비상정지 잠금을 우회하지 않는다. BU03/BU04 설정을 바꾸기 전에는 AT 설정 백업을 보존한다.

`pi-runtime/`은 미배포 MOCK 후보이며 `pi-runtime/pi-export-reference/`는 제공받은 Pi 원본 소스의 보존본이다. 보존본에는 실제 GPIO 코드가 있으나 현재 Pi 설치본과 일치하는지 확인되지 않았다. 이 둘을 자동으로 설치하거나 덮어쓰지 않는다.

## 1. Pi 접속과 기존 센싱 확인

PC PowerShell에서 Pi에 접속한다. 아래 IP는 마지막 확인값이다. 바뀌었다면 Pi에서 `hostname -I`로 주소를 확인하고 인증서도 해당 IP에 맞춰 확인한다.

```powershell
ssh capstone@192.168.43.212
```

LiDAR·BU04를 Pi에 연결하고 BU03 태그 전원을 켠 뒤 Pi에서 실행한다.

```bash
ss -ltnp '( sport = :8765 )'
curl -sS --max-time 2 -o /dev/null -w '센싱 HTTP 상태: %{http_code}\n' http://127.0.0.1:8765/api/health
ps -ww -eo pid,ppid,args | grep -E '[c]art.py|[s]martcart_runner|[s]ensing.py'
```

HTTP 200과 기존 센싱 프로세스가 있으면 그대로 사용한다. 마지막 사용자 보고에서는 이미 실행 중이었다. HTTP 200은 센서 측정이나 위치가 유효하다는 뜻은 아니다.

응답이 없으면 잠금 보유자를 확인한다. 일치하는 잠금 파일이 없다는 출력도 확인 결과다.

```bash
fuser -v /tmp/smartcart-runner-*/live.lock /tmp/smartcart-runner-*/8765.lock
```

8765 서버와 잠금 보유자가 모두 없을 때만 다음 명령을 **한 번** 실행하고 해당 터미널을 유지한다.

```bash
bash "$HOME/smartcart/SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh"
```

이 명령은 실측 센싱 전용이다. 단일 BU04 수평거리 가정과 장착·정확도는 검수 전이다. 잠금 오류가 나면 파일을 삭제하지 말고 위 명령으로 기존 프로세스를 확인한다.

## 2. 기존 코드와 별도로 가상 화면 실행

아래는 GitHub `main`의 `pc-development/virtual-lab-18084/`를 새 폴더에서 실행하는 절차다. 실측 센서는 읽지만 `MockBackend`만 사용하므로 실제 모터를 구동하지 않는다. 기존 Pi 설치 코드를 변경하지 않으며, 이 GitHub 실행본의 Pi 적용·화면 검증은 아직 확인되지 않았다. 시험 중 실제 모터 전원은 차단한다.

Pi에 **별도 SSH 터미널**로 접속해 다음을 실행한다. GitHub 접근과 `git`, Python `aiohttp`가 필요하다. 아래 명령은 새 폴더에만 복제하며, 이후 실행도 이 터미널에서 계속한다.

```bash
python3 -c "import aiohttp; print('aiohttp 확인 완료')"
task_dir="$(mktemp -d "$HOME/smartcart-github.XXXXXXXX")" &&
  git clone --depth 1 https://github.com/Protocol314/smartcart.git "$task_dir" &&
  git -C "$task_dir" rev-parse HEAD &&
  printf '복제 폴더: %s\n' "$task_dir"
```

`aiohttp`가 없으면 Pi에서 `sudo apt install python3-aiohttp`로 설치한다. 복제나 패키지 확인이 실패하면 오류를 확인하고 다음 단계로 넘어가지 않는다.

출력된 커밋과 복제 폴더를 기록한다. GitHub `main`은 이후 갱신될 수 있으며, 별도 초안 PR의 코드는 병합 전까지 포함되지 않는다. 센싱이 실행 중이라면 같은 터미널에서 상태를 조회한다.

```bash
bash "$task_dir/pi-runtime/check_pi.sh" --sensors
```

진단은 LiDAR·BU04·추적 상태를 5회 표시하며 위치와 접속키를 출력하지 않는다. 복제본에는 장치 설정·ROS 빌드 결과가 없으므로 환경 항목이 없다고 표시될 수 있다. `1m 이내`는 근접점 개수로 무효 빔 수와 별개다. 장치 연결 표시만으로 측정 유효·ID 일치·위치 정확도를 판정하지 않는다.

다음 인증서 경로는 사용자가 이전에 확인한 Pi 경로다. 인증서 검증 결과가 `server.crt: OK`일 때만 진행한다.

```bash
task_cert="$HOME/smartcart/smartcart-virtual-lan-cert"
openssl verify -CAfile "$task_cert/ca.crt" -verify_ip 192.168.43.212 "$task_cert/server.crt"
ss -ltnp '( sport = :18085 )'
```

18085 포트를 사용하는 프로세스가 없으면 같은 터미널에서 실행한다. 인증서 경고는 우회하지 않으며 사용 중인 포트의 출처를 모르면 강제 종료하지 않는다.

```bash
python3 -u "$task_dir/pc-development/virtual-lab-18084/virtual_lan_server.py" \
  --address 192.168.43.212 --port 18085 \
  --cert "$task_cert/server.crt" --key "$task_cert/server.key"
```

터미널에서 숫자 4자리 화면 접속키를 두 번 정한다. 입력 중에는 숫자가 보이지 않는다. 서버가 출력하는 **조이스틱 연결키**는 화면 접속키와 별개다. 둘 다 채팅·로그에 공유하지 않는다. 화면은 신뢰하는 사설망에서만 사용한다.

## 3. 센서와 조이스틱 시험

Chrome에서 `https://192.168.43.212:18085/`에 접속해 화면 접속키로 로그인한다. 왼쪽 연결란에는 터미널의 **조이스틱 연결키**를 입력한다. `가상 출력 · 실제 모터 구동 안 됨`을 확인한 뒤, 화면 하단 센서 설정의 `SOURCE`를 `Pi 센싱 서버`로 두고 센서 상태를 확인한다. 가상 주행 설정에서는 `실측 LiDAR + BU04`를 선택할 수 있다.

방향 버튼을 놓은 뒤 활성화가 풀리는 원인은 아직 미확인이다. Chrome 창을 활성 상태로 두고 `주행 활성화` 후 방향 버튼을 짧게 눌렀다 놓고, 풀리면 왼쪽 연결란 옆의 상태 문구를 기록한다. 창 포커스 해제나 통신 지연 등은 안전 정지를 일으킬 수 있다.

UWB 마름모의 화면 아래 1.25m 이동은 **표시만** 바꾸는 임시 보정이다. 측정·추적·제어 좌표와 실제 정확도는 바꾸지 않는다. 가상 자율주행은 무효·만료된 센서나 모호한 추적을 거부한다. 가상 수동 방향 조작은 센서 기반 장애물 자동 정지를 제공하지 않는다. 어떤 가상 시험도 실제 주행 승인이나 장애물 회피 검수가 아니다.

## 종료와 참고

- 가상 서버는 직접 시작한 터미널에서 `Ctrl+C`로 종료한다. 센싱은 별도 터미널에서 실행된다.
- UWB 추종 [초안 PR #1](https://github.com/Protocol314/smartcart/pull/1)은 2026-11-24 기준 미병합이다. 의존 파일이 누락됐고 LiDAR 감시 OFF·기본 GPIO 설정 등 검토가 필요하므로 실행하지 않는다.
- Pi MOCK 후보의 환경·ROS 빌드 준비는 `pi-runtime/README.md`의 `check_pi.sh`와 `prepare_ros.sh`를 따른다. 실제 Ubuntu·ROS 빌드와 실물 검수는 미완료다.
- 화면 인증서·키 오류는 IP·CA 신뢰·파일 경로를 확인한다. 잠금 파일이나 장치 설정·백업을 임의로 지우지 않는다. 상세 이력은 `HANDOFF.md`와 `docs/`에 보관한다.
