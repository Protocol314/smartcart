"""GPIO를 열지 않고 차동 명령과 회전 방향을 확인하는 계산 예시."""
from .control import mix

print('모터 명령 계산 예시 · GPIO 미개방 · 실물 주행 아님')
for name, forward, turn in (('전진', 1, 0), ('후진', -1, 0),
                            ('제자리 좌회전', 0, 1), ('제자리 우회전', 0, -1)):
    left, right = mix(forward, turn)
    print(f'{name}: 차체 기준 왼쪽 {left:+.1f}, 오른쪽 {right:+.1f}')
print('각 드라이버의 전진 극성은 실물 확인 전 미배정입니다.')
