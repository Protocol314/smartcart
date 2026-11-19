"""설치본에서도 고정된 기존 진입점만 실행하는 자식 프로세스."""
import sys


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ("sensing", "uwb"):
        print("실행 대상은 sensing 또는 uwb입니다", file=sys.stderr)
        return 2
    name = argv.pop(0)
    if name == "sensing":
        from smartcart_sensing.app import main as entry
    else:
        from smartcart_uwb.ros_adapter import main as entry
    return entry(argv)


if __name__ == "__main__":
    raise SystemExit(main())
