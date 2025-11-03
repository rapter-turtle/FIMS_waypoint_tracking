
import math

def calculate_s_point(my_lat, my_lon, a_lat, a_lon, distance=30.0):
    """
    내 위치 기준으로 A의 반대 방향 distance(m) 떨어진 S 지점 계산
    
    Args:
        my_lat: 내 위치 위도
        my_lon: 내 위치 경도
        a_lat: A 지점 위도
        a_lon: A 지점 경도
        distance: S와 내 위치 사이의 거리 (미터, 기본값 30m)
    
    Returns:
        tuple: (s_lat, s_lon) S 지점의 위도, 경도
    """
    # A에서 내 위치로의 방향 벡터
    dx = my_lon - a_lon
    dy = my_lat - a_lat
    
    # 벡터의 길이
    length = math.sqrt(dx * dx + dy * dy)
    
    if length == 0:
        raise ValueError("내 위치와 A 지점이 동일합니다.")
    
    # 단위 벡터 (A에서 내 위치 방향)
    ux = dx / length
    uy = dy / length
    
    # 위도/경도를 미터로 변환하는 근사값
    # 위도 1도 ≈ 111,000m
    # 경도 1도 ≈ 111,000m * cos(위도)
    lat_per_meter = 1.0 / 111000.0
    lon_per_meter = 1.0 / (111000.0 * math.cos(math.radians(my_lat)))
    
    # S 지점 계산 (내 위치에서 A의 반대 방향으로 distance만큼)
    s_lat = my_lat + uy * distance * lat_per_meter
    s_lon = my_lon + ux * distance * lon_per_meter
    
    return s_lat, s_lon


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Haversine 공식을 사용한 두 지점 간 거리 계산 (미터)
    
    Args:
        lat1, lon1: 첫 번째 지점의 위도, 경도
        lat2, lon2: 두 번째 지점의 위도, 경도
    
    Returns:
        float: 두 지점 간 거리 (미터)
    """
    R = 6371000  # 지구 반지름 (미터)
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c


# 사용 예제
if __name__ == "__main__":
    # 예제 좌표
    my_lat = 37.5660
    my_lon = 126.9775
    a_lat = 37.5665
    a_lon = 126.9780
    
    # S 지점 계산
    s_lat, s_lon = calculate_s_point(my_lat, my_lon, a_lat, a_lon, distance=30.0)
    
    print("=" * 50)
    print("S 지점 계산 결과")
    print("=" * 50)
    print(f"내 위치: ({my_lat:.6f}, {my_lon:.6f})")
    print(f"A 지점:  ({a_lat:.6f}, {a_lon:.6f})")
    print(f"S 지점:  ({s_lat:.6f}, {s_lon:.6f})")
    print()
    
    # 거리 검증
    dist_s_to_my = haversine_distance(s_lat, s_lon, my_lat, my_lon)
    dist_my_to_a = haversine_distance(my_lat, my_lon, a_lat, a_lon)
    
    print(f"S → 내 위치 거리: {dist_s_to_my:.2f}m (목표: 30m)")
    print(f"내 위치 → A 거리: {dist_my_to_a:.2f}m")
    print("=" * 50)
