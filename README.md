# SmartCart Pi·PC 통합 프로젝트

Raspberry Pi 4 / Ubuntu 22.04 ARM64 / ROS 2 Humble을 사용하는 스마트카트 프로젝트다. 현재 시험 구성은 C1 LiDAR와 단일 BU04를 손잡이 쪽에 설치하고, BU03 태그를 사용자가 소지하는 형태다.

## 실제 구동의 현재 상태

다른 사용자가 기존 Pi 카트 코드의 동작을 확인했다고 보고했으나, 새 `pi-runtime/` 사본은 **Pi에 배포하거나 실제 GPIO로 검수한 버전이 아니다**. 새 사본의 `manual.sh`는 MOCK 전용이며 실제 GPIO를 잠근다. 이전 Pi 수동 서버는 LiDAR·UWB를 표시하지만 센서 이상 시 모터 차단이나 실제 자율 추종·장애물 회피가 연결되어 있지 않다. 기존 Pi에 2초 PWM 응답 감시가 적용됐는지도 확인되지 않았다.

전진 극성, 모터 전원 차단·비상정지 피드백, LiDAR 장착·정지 범위, BU04 거리 모델과 사람 위치 정확도, 정지·재시작 조건은 실물 검수가 필요하다. 화면상 PWM 확인은 엔코더가 없는 실제 바퀴 움직임을 뜻하지 않는다. 무효 센서나 시간 초과의 구동 잠금을 우회하지 않는다.

## 작업 폴더

| 경로 | 역할 |
| --- | --- |
| [`pc-development/`](pc-development/README.md) | Pi 내보내기와 최신 가상 안전 수정본을 기준으로 한 PC 개발 사본 |
| [`pi-runtime/`](pi-runtime/README.md) | Ubuntu ARM64 MOCK 실행 후보와 제공받은 Pi 원본 소스 보존본 |
| [`pc-development/virtual-lab-18084/`](pc-development/virtual-lab-18084/README.md) | 최신 18084 가상 화면·버튼 진단 소스 |
| [`pc-development/integration-lab/`](pc-development/integration-lab/README.md) | 통합 센싱·추적 개발 코드와 Python·JS·BU03/BU04 C 회귀 기준 |
| [`pi-runtime/pi-export-reference/`](pi-runtime/pi-export-reference/README.md) | 제공받은 Pi 원본의 소스 기준본. 설치본과 해시 비교 전 |
| [`runbook/`](<runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.md>) | 실제 구동 구성, 센싱 및 별도 가상 시험 절차 |
| [`HANDOFF.md`](HANDOFF.md) | 최신 작업 결과, 미확인 항목과 이전 기록 |

`pc-development/`과 `pi-runtime/`은 독립 사본이다. PC에서 수정한 내용이 Pi 사본에 자동 반영되지 않는다. 구성 당시 파일별 해시는 각 `SOURCE-MANIFEST.json`에 있다. Pi 내보내기 원본 ZIP과 압축 해제본에는 개인 설정·인증서가 포함되어 Git에서 제외한다.

## 최근 Pi 확인과 남은 작업

재부팅 후 사용자가 Pi의 센싱 서버 `127.0.0.1:8765`에서 HTTP 200과 단일 BU04 수집·ROS 센싱 프로세스 및 정상 잠금 보유를 보고했다. 서버 응답은 LiDAR·BU04 측정 유효성을 의미하지 않는다. 이전 18084 화면은 실측 센서를 보여 주지만 출력은 MOCK이다. Windows Chrome에서 방향 버튼 해제 후 활성화가 풀리는 원인은 미확인이며, `pc-development/virtual-lab-18084/`는 상태 표시와 화면 배치를 보완한 진단 후보이다. Pi에 후보 적용 및 실제 브라우저 검증 결과는 확인되지 않았다. UWB 1.25m 이동은 표시 전용이다.

## 가상 검증 및 남은 작업

새 사본은 LiDAR·BU04 무효나 만료 시 가상 출력 정지와 2초 PWM 응답 감시를 포함한다. 두 센서의 HTTP 조회를 병렬화한 뒤 합성 HTTP·WebSocket 시험은 6회 통과했으며, 센서 만료 제한은 유지한다. Ubuntu/ROS 빌드와 실제 Pi 실측은 아직 확인하지 않았다. Pi에 후보를 배치한 후 `bash check_pi.sh`로 빌드 준비 상태를, 기존 센싱 서버가 실행 중이면 `bash check_pi.sh --sensors`로 상태를 읽기 전용 확인할 수 있다.

마지막 사용자 보고에서 LiDAR는 720개 중 66개 빔이 무효였고 BU04는 USB 연결 상태이나 측정 표본이 없었다. 이후 사용자가 화면에 센서가 보인다고 보고했으나 유효 측정·ID 일치와 위치 정확도는 확인되지 않았다. 센서 복구와 가상 통합 안정성 확인 후, 실제 안전 상태 공급자와 현장 검수를 완료해야 한다. 자세한 순서는 [운영 런북](<runbook/Smart-Cart Raspberry Pi 4 기본 운영 런북.md>)을 따른다.
