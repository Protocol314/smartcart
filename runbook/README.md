# 운영 런북 관리

운영 본문은 [Markdown](<Smart-Cart Raspberry Pi 4 기본 운영 런북.md>), 열람·코드 복사본은 [HTML](<Smart-Cart Raspberry Pi 4 기본 운영 런북.html>)이다. MD를 수정한 뒤 프로젝트 루트에서 HTML을 다시 생성하고 검사한다.

```powershell
python -X utf8 runbook/build_runbook.py
python -X utf8 runbook/build_runbook.py --check
python -X utf8 runbook/build_runbook.py --self-test
```

외부 패키지나 CDN 없이 생성한다. `--check`는 전체 HTML과 원본의 일치를 검증한다. 과거 작업 이력은 프로젝트의 `HANDOFF.md`에 보관한다.
