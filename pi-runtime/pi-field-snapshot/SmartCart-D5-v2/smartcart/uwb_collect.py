"""설치하지 않은 소스 폴더에서도 좌우 UWB 수집기를 실행한다."""
from smartcart_uwb.app import main

if __name__ == "__main__":
    raise SystemExit(main())
