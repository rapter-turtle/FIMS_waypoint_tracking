
import math

def calculate_s_point_utm(my_easting, my_northing, a_easting, a_northing, distance=30.0):
    """
    UTM 좌표계에서 내 위치 기준으로 A의 반대 방향 distance(m) 떨어진 S 지점 계산
    
    Args:
        my_easting: 내 위치 UTM Easting 좌표 (미터, 동쪽 방향)
        my_northing: 내 위치 UTM Northing 좌표 (미터, 북쪽 방향)
        a_easting: A 지점 UTM Easting 좌표 (미터, 동쪽 방향)
        a_northing: A 지점 UTM Northing 좌표 (미터, 북쪽 방향)
        distance: S와 내 위치 사이의 거리 (미터, 기본값 30m)
    
    Returns:
        tuple: (s_easting, s_northing) S 지점의 UTM Easting, Northing 좌표 (미터)
    """
    # A에서 내 위치로의 방향 벡터
    d_easting = my_easting - a_easting
    d_northing = my_northing - a_northing
    
    # 벡터의 길이
    length = math.sqrt(d_easting * d_easting + d_northing * d_northing)
    
    if length == 0:
        raise ValueError("내 위치와 A 지점이 동일합니다.")
    
    # 단위 벡터 (A에서 내 위치 방향)
    u_easting = d_easting / length
    u_northing = d_northing / length
    
    # S 지점 계산 (내 위치에서 A의 반대 방향으로 distance만큼)
    # UTM 좌표계에서는 미터 단위이므로 직접 계산
    s_easting = my_easting + u_easting * distance
    s_northing = my_northing + u_northing * distance
    
    return s_easting, s_northing


def euclidean_distance_utm(easting1, northing1, easting2, northing2):
    """
    UTM 좌표계에서 유클리드 거리 계산 (미터)
    
    Args:
        easting1, northing1: 첫 번째 지점의 UTM Easting, Northing 좌표
        easting2, northing2: 두 번째 지점의 UTM Easting, Northing 좌표
    
    Returns:
        float: 두 지점 간 거리 (미터)
    """
    d_easting = easting2 - easting1
    d_northing = northing2 - northing1
    return math.sqrt(d_easting * d_easting + d_northing * d_northing)


# 사용 예제
if __name__ == "__main__":
    # 예제 UTM 좌표 (예: 한국 서울 지역 - UTM Zone 52N)
    # 실제 사용시에는 위도/경도를 UTM으로 변환한 값을 사용
    my_easting = 323394.0    # UTM Easting 좌표 (미터, 동쪽)
    my_northing = 4161274.0  # UTM Northing 좌표 (미터, 북쪽)
    a_easting = 323450.0     # A 지점 UTM Easting 좌표 (미터, 동쪽)
    a_northing = 4161330.0   # A 지점 UTM Northing 좌표 (미터, 북쪽)
    
    # S 지점 계산
    s_easting, s_northing = calculate_s_point_utm(my_easting, my_northing, 
                                                  a_easting, a_northing, 
                                                  distance=30.0)
    
    print("=" * 50)
    print("S 지점 계산 결과 (UTM 좌표계)")
    print("=" * 50)
    print(f"내 위치: (E:{my_easting:.1f}, N:{my_northing:.1f}) m")
    print(f"A 지점:  (E:{a_easting:.1f}, N:{a_northing:.1f}) m")
    print(f"S 지점:  (E:{s_easting:.1f}, N:{s_northing:.1f}) m")
    print()
    
    # 거리 검증
    dist_s_to_my = euclidean_distance_utm(s_easting, s_northing, 
                                          my_easting, my_northing)
    dist_my_to_a = euclidean_distance_utm(my_easting, my_northing, 
                                          a_easting, a_northing)
    
    print(f"S → 내 위치 거리: {dist_s_to_my:.2f}m (목표: 30m)")
    print(f"내 위치 → A 거리: {dist_my_to_a:.2f}m")
    print("=" * 50)
