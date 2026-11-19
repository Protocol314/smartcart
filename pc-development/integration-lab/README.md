# PC 통합 개발·회귀 기준본

기존 `smartcart/`의 센싱·추적·가상 모터 및 웹 개발 코드와 회귀 시험, `components/c_key_core/`의 BU03/BU04 네이티브 파서, `tests/`의 호스트 C 시험을 보존한 PC 개발 기준본이다. `pc-development/`의 Pi 기반 개발 후보 및 `virtual-lab-18084/`는 별도 버전이며 자동 동기화되지 않는다.

개인 설정·장치 백업·인증서·로그와 실제 사용자의 스캔 샘플과 이전 배포용 파일 해시 명세는 포함하지 않는다. C 시험 실행 스크립트의 줄바꿈은 Ubuntu에서 실행할 수 있도록 LF로 정리했다. `smartcart/README.md` 등 내부 문서에는 과거 장치 구성 및 경로가 남아 있으므로 최신 운영 절차는 저장소의 `runbook/`과 HANDOFF를 따른다. 일부 브라우저·재생 시험은 제외된 실측 샘플이 필요하고, ROS·Pi 전용 시험은 별도 환경에서 실행한다.

Linux에서 C 회귀 시험은 이 폴더에서 `bash tests/run_host_tests.sh`를 실행한다. Python 가상 시험은 `cd smartcart` 후 `python3 -m unittest discover -s tests -p 'test_drive_supervisor.py'` 등을 사용한다. 실제 GPIO 출력은 별도 현장 검수 전 잠긴 상태다. 이 폴더는 Pi에 자동 배포하지 않는다.
