# SmartCart Raspberry Pi 4 시연 운영 런북

Pi 4 · Ubuntu 22.04 · ROS 2 Humble · C1 LiDAR · BU04 · BU03 태그

## 1. SSH 접속

Windows PowerShell에서 Pi에 접속한다. 핫스팟이 바뀌면 Pi에서 `hostname -I`로 확인한 주소를 사용한다.

```powershell
ssh capstone@192.168.43.212
```

## 2. 센싱 확인

이후 명령은 Pi SSH 터미널에서 실행한다. LiDAR와 BU04를 연결하고 BU03의 전원을 켠 뒤 센싱 상태를 확인한다.

```bash
curl -fsS --max-time 2 http://127.0.0.1:8765/api/health
```

응답이 오면 기존 서버를 사용한다. 응답이 없으면 `ss -ltnp '( sport = :8765 )'`로 포트를 확인한다. 기존 실행기나 잠금 보유자가 없고 Pi에 `sensors.sh`가 있는 경우에만 시작한다.

```bash
cd "$HOME/smartcart" && bash sensors.sh
```

잠금 오류가 나면 소유 프로세스를 확인한다. HTTP 응답과 별도로 LiDAR 갱신·미관측 구간과 BU04 측정 유효·ID 일치를 확인한다.

## 3. 조종 서버

**`real.sh`는 실제 GPIO를 사용한다.** 센서 이상·ID 불일치·시간 초과 시 정지 및 재활성화 조건, 2초 PWM 응답 감시, 비상정지와 모터 극성을 현장에서 확인하기 전까지 실행을 보류한다. Pi의 추종 코드 일부는 GitHub 검토본과 달라 해당 잠금의 적용 여부가 확인되지 않았다.

안전 검수를 마친 뒤 `ss -ltnp '( sport = :8081 )'`로 기존 서버가 없는지 확인하고, 별도 Pi SSH 터미널에서 실행한다.

```bash
cd "$HOME/smartcart" && bash real.sh
```

주소를 생략하면 서버가 사설 IPv4를 선택한다. `RFC1918 private IPv4 required`가 나오면 `0.0.0.0` 대신 Pi에 할당된 주소를 사용한다. 현재 주소가 `192.168.43.212`라면 같은 디렉토리에서 `bash real.sh 192.168.43.212`로 지정할 수 있다.

같은 핫스팟의 PC나 휴대폰에서 서버가 출력한 HTTPS 주소로 접속한다. 현재 주소 예시는 `https://192.168.43.212:8081/`이다. 인증서 경고가 나오면 주소와 인증서 신뢰 설정을 확인한다.

센서·응답·비상정지에 이상이 있으면 구동을 중단하고 모터 전원을 차단한다. 직접 시작한 서버는 해당 터미널에서 `Ctrl+C`로 종료한다.
