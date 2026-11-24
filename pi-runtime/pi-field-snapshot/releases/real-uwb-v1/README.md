# SmartCart Real UWB v1.0.0

라즈베리파이 4의 기존 GPIO 드라이버를 사용하는 실차용 **시험 버전**이다. 실물 주행을 검증한 배포판은 아니다. 기본 백엔드는 gpio이며, mock은 GPIO 없이 검사할 때만 선택한다.

## 동작

- 기존 8081 화면의 사용자 추종·자율주행 버튼을 사용한다.
- `/api/dual-uwb`에서 단일 BU04 측정·태그 ID·표본 순번·수명을 직접 검사한다. LiDAR 대상 확정과 `/api/tracking`에 의존하지 않는다.
- 추종거리는 **BU04의 측정거리(높이 차이를 포함할 수 있는 거리)**다. 차체 기준 지상거리로 표시하지 않는다. 2.25m 이상이면 출발하고, 2.00m 이하이면 정지한다.
- 전진 명령은 0 또는 0.10 이상이므로 기존 0.08 데드존과 충돌하지 않는다. PWM 상한은 기존 시험과 같은 30/255이며 출발에 충분한지는 모터·하중에서 확인해야 한다. 출발이 안 된다고 제한을 자동으로 올리지 않는다.
- 센서가 유효한 서로 다른 표본 3개를 보낸 후 시작할 수 있다. 무효·표본 만료·큰 위치 점프·전방 범위 이탈은 정지와 재활성화를 요구한다. 같은 표본을 재조회해 수명을 연장하지 않는다.
- 기존 GPIO 핀·극성(-1,-1)·모터 잠금·가감속·방향전환 대기·명령/제어 워치독을 그대로 사용한다.
- `applied_seq`는 기존 백엔드의 `write`가 성공한 후에만 기록한다. 이는 소프트웨어 PWM 처리 확인이며 실제 바퀴 회전 측정이 아니다.
- **LiDAR 장애물 회피, 물리 전원 차단 피드백, 엔코더 속도 제어, 검증된 제동거리는 없다.** 비상 정지 수단을 확보하고 바퀴를 띄운 상태에서 극성·STOP부터 확인한다.

## 설치와 실행

기존 8081/18084 조종 서버 창에서 Ctrl+C로 종료한다. 센싱 8765는 유지할 수 있다. 압축 해제 폴더에서 `python3 install.py`를 실행한다. `~/smartcart/releases/real-uwb-v1`에 별도로 설치하고 `real.sh`와 `virtual.sh`만 새 실행기로 바꾼다. 이전 실행기는 archive에 백업한다. 기존 프로젝트·ROS 빌드·장치 설정·UWB 보정·인증서·와이파이 설정은 덮지 않는다.

센싱 창:

```bash
bash ~/smartcart/sensors.sh
```

조종 창:

```bash
bash ~/smartcart/real.sh 172.20.10.4 gpio
```

IP를 생략하면 wlan0, 다음 eth0의 사설 IPv4를 선택한다. 명시적 mock 검사:

```bash
bash ~/smartcart/real.sh 172.20.10.4 mock
```

화면: `https://현재-Pi-IP:8081/`. 화면 접속키를 실행 터미널에서 두 번 설정한 뒤 별도로 출력되는 조이스틱 키를 입력한다. 주행 활성화 → 사용자 추종 ON → 자율주행 ON 순서다. 수동 조작·OFF·STOP·포커스 손실·연결 끊김은 자율 모드를 해제한다. 정지 뒤 자동으로 재출발하지 않는다.

## 핫스팟과 인증서

10/8, 172.16/12, 192.168/16 사설 주소를 허용한다. 기존 인증서가 현재 IP와 맞으면 그대로 사용한다. 기존 ca.key가 있으면 같은 CA로 새 IP의 서버 인증서를 별도 생성한다. ca.key가 없으면 `~/smartcart/certs/real-v1` 안에 별도 CA를 만들며, **새 ca.crt를 브라우저 단말에 설치·신뢰해야 한다.** TLS 경고를 우회하지 않는다. CA 파일 경로는 서버가 출력한다. ca.key와 server.key는 공유하지 않는다. 핫스팟 DHCP 주소를 고정하는 기능은 아니다.

## 진단·보정

서버 상태에는 실제 선택한 백엔드, BU04 측정거리·각도·수명, 정지 사유, 전진·회전 명령과 PWM을 표시한다. 무효 상태를 정상값으로 바꾸거나 관측용 `provisional`, `drive_authorized` 값을 변조하지 않는다. 기존 관측기와 별개인 UWB 단독 시험 제어기로서 인증된 조종자의 활성화와 유효 입력을 요구한다.

거리·각도 정확도, 설치 방향, 출발 PWM, 실제 바퀴 극성·정지·하중 영향은 미검증이다. `follow_config.json`의 angle_sign=-1은 기존 BU04 파서의 부호 규칙을 따른다. 설정 변경 후 재시작한다. PWM 상한 30 및 표본 수명 200ms를 넘는 설정은 이 버전에서 거부한다.

실행기 복구는 설치 시 표시된 archive 폴더의 virtual.sh를 `~/smartcart/virtual.sh`로 복사하면 된다. 버전 폴더와 백업은 삭제하지 않는다.

## 새 CA가 필요한 경우

서버가 사용하는 공개 CA 사본은 `~/smartcart/SmartCart_Real_UWB_v1_CA.crt`다. 개인키는 포함하지 않는다. Windows PowerShell에서:

```powershell
scp capstone@172.20.10.4:~/smartcart/SmartCart_Real_UWB_v1_CA.crt "$HOME\Downloads\SmartCart_Real_UWB_v1_CA.crt"
Import-Certificate -FilePath "$HOME\Downloads\SmartCart_Real_UWB_v1_CA.crt" -CertStoreLocation Cert:\CurrentUser\Root
```

자신의 Pi가 만든 CA인지 확인하고 설치한다. iPhone에서도 해당 공개 인증서를 받아 프로파일을 설치한 뒤 설정 → 일반 → 정보 → 인증서 신뢰 설정에서 해당 CA의 신뢰를 켠다. 단말마다 한 번 필요하며 CA가 바뀌면 다시 설치한다. 주소가 바뀌면 현재 주소로 서버를 재시작한다. 인터넷 외부 공개나 고정 IP 설정은 하지 않는다.

## 검증 범위

2026-11-19 소프트웨어 검사: Python 31개 통과, JavaScript 검사 스크립트 2개 통과. HTTPS 로그인부터 WebSocket 인증·중립 활성화·UWB 명령·PWM 쓰기 확인·STOP까지 연결 검사했다. 설치 중 두 번째 실행 파일 교체가 실패할 때 이전 실행 파일로 복구되는 것도 검사했다.

검사는 기존 모터 모듈의 MockBackend와 생성한 UWB 입력으로 실행했으며 GPIO 출력은 없었다. Pi의 패키지·USB·실측 UWB 갱신률·GPIO 권한·실제 회전 극성·모터의 출발 PWM·제동 거리는 검증하지 못했다. 다운로드 파일에는 ROS 전체나 기존 GPIO 드라이버가 중복 포함되지 않으며 현재 `~/smartcart` 설치본이 필요하다.
