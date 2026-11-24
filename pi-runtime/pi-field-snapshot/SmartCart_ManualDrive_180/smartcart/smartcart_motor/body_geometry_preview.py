"""측정한 직사각형 차체와 LiDAR 상대 위치의 기하학적 크기만 계산한다.

계산 결과는 정지거리·장애물 센서 범위·보정·구동 승인이 아니다.
"""
import math


def _finite(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(name + ' 범위가 잘못되었습니다')
    return float(value)


def preview_body_envelope(length_m, width_m, lidar_from_front_m, lidar_left_of_center_m):
    """직사각형 네 꼭짓점 중 LiDAR에서 가장 먼 점까지의 거리만 반환한다.

    LiDAR가 차체 경계 안에 있는 경우로 한정한다. 손잡이·바퀴·돌출물 및
    회전 중 궤적은 별도 실측해야 하며 반환값을 정지 반경으로 사용하지 않는다.
    """
    length = _finite(length_m, '차체 길이', .1, 5)
    width = _finite(width_m, '차체 폭', .1, 5)
    front = _finite(lidar_from_front_m, '앞끝에서 LiDAR 거리', 0, length)
    left = _finite(lidar_left_of_center_m, 'LiDAR 좌우 오프셋', -width / 2, width / 2)
    corner_radius = max(math.hypot(x - front, y - left)
                        for x in (0, length) for y in (-width / 2, width / 2))
    return {
        'schema': 'smartcart.body_geometry_preview.v1',
        'body_corner_radius_m': corner_radius,
        'braking_distance_included': False,
        'protrusions_included': False,
        'swept_rotation_included': False,
        'geometry_verified': False,
        'drive_authorized': False,
    }
