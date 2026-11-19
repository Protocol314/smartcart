# Raspberry Pi 실행 구성

이 폴더는 2026-11-19 Pi 내보내기와 그 위의 가상 안전 수정본을 기준으로 만든 소스 사본이다. `pc-development/`는 같은 시점의 개발 사본이다. 두 폴더는 독립적이며 한쪽을 수정해도 다른 쪽에 자동 반영되지 않는다.

`SmartCart-D5-v2/`는 ROS 센싱·UWB·LiDAR, `SmartCart_ManualDrive_180/`은 수동 조종 화면과 센서 감시를 포함한 MOCK 출력, `smartcart_ws/src/`는 LiDAR 드라이버 소스다. `SOURCE-MANIFEST.json`에는 구성한 시점의 소스 해시가 있다.

현장 설정·USB 식별자·실측 자료·TLS 개인키·인증서·이전 ROS 빌드 결과는 포함하지 않는다. 실제 Pi와 장치의 설정은 기존 원본을 보존한 상태로 따로 확인하고, 검수 전 값을 새 장치에 복사하지 않는다. `SMARTCART_CERT_DIR`은 Pi에서 관리하는 인증서 폴더를 가리켜야 한다.

Ubuntu 22.04 ARM64에서 ROS 2 Humble, colcon, gcc, python3, aiohttp 및 필요한 ROS 패키지를 준비한 후 `bash prepare_ros.sh`로 현재 위치에 ROS 작업공간과 BU04 네이티브 라이브러리를 새로 빌드한다. 실제 장치 설정이 올바르게 준비된 경우에만 `bash sensors.sh`로 센싱을 시작한다. 화면은 `SMARTCART_CERT_DIR`을 지정한 상태에서 `bash manual.sh 192.168.43.X`로 실행한다. 이 화면은 센서 감시가 있는 MOCK 전용이며 실제 GPIO가 잠겨 있다. 이미 같은 포트의 서버가 있으면 중복 실행하지 않는다.

LiDAR와 BU04 조회를 병렬 처리하도록 변경한 가상 HTTP·WebSocket 시험은 6회 통과했다. 센서 만료 기준은 늘리지 않았으며 실제 Pi의 지연·미관측 빔·BU04 측정 이상은 아직 확인하지 않았다. 이전 실제 GPIO 서버는 이 폴더로 대체·배포하지 않았다.

장치와 GPIO를 열지 않고 Pi의 빌드 준비 상태를 보려면 `bash check_pi.sh`를 실행한다. ROS 환경·필요 패키지·현장 설정·빌드 산출물의 존재만 확인한다. `bash prepare_ros.sh`로 빌드한 뒤 동일 명령으로 다시 확인한다. 기존 8765 센싱이 별도로 실행 중일 때만 `bash check_pi.sh --sensors`로 LiDAR·BU04·추적 상태를 5회 읽는다. 위치·태그 식별자·접속키는 출력하지 않는다. 센싱 서버를 새로 시작하거나 장치 설정을 변경하지 않으며, 실제 주행 승인이나 거리 정확도 검수가 아니다.

`pi-export-reference/`는 사용자가 제공한 ZIP의 원본 코드 중 현장 설정과 비밀을 제외하고 보존한 기준본이다. 실제 GPIO 수동 코드가 포함되어 있으나 Pi의 현재 설치본과 해시 비교가 되지 않았고 안전 검수가 완료된 버전이 아니다. 이 폴더와 상위 MOCK 후보는 별개이며 자동으로 설치하거나 동기화하지 않는다.
