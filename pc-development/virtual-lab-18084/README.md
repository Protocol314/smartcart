# 18084 가상 조이스틱 개발본

2026-11-19 레이아웃·연결 진단 후보의 소스다. 독립된 `virtual_lan_server.py`는 `MockBackend`만 사용하며 실제 GPIO 또는 모터를 구동하지 않는다. 실측 센싱 서버 `127.0.0.1:8765`를 읽어 화면에 표시한다. 새 `pi-runtime/` 후보 및 기존 Pi 실제 수동 서버와 별개다.

UWB 마름모를 화면 아래 1.25m 이동하는 것은 임시 표시 보정이며 실제 측정·추적·제어 좌표에는 적용되지 않았다. Pi에서 방향 버튼 해제 후 활성화가 풀리는 현상은 원인이 확인되지 않았다. 연결란 옆 상태 문구를 확인한다. 화면 배치의 실제 기기 검증도 미완료다.

웹 회귀 시험은 이 폴더에서 `node tests/test_manual_controls.cjs`, `node tests/test_virtual_autonomy.cjs`, `node tests/test_virtual_live.cjs`, `node tests/test_display_autonomy.cjs`로 수행한다. 인증서·개인키 및 현장 설정은 포함하지 않는다. Pi 실행 방법은 저장소 상위의 운영 런북을 따른다.
