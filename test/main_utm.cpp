#include <iostream>
#include <cmath>
#include <stdexcept>
#include <iomanip>

/**
 * UTMStartPosition 클래스: UTM 좌표계에서 시작점 계산을 위한 클래스
 */
class UTMStartPosition {
public:
    /**
     * UTM 좌표를 나타내는 구조체
     */
    struct UTMPoint {
        double easting;   // UTM Easting 좌표 (동쪽 방향, 미터)
        double northing;  // UTM Northing 좌표 (북쪽 방향, 미터)
        
        UTMPoint(double easting_coord, double northing_coord) 
            : easting(easting_coord), northing(northing_coord) {}
        
        UTMPoint() : easting(0.0), northing(0.0) {}
    };

    /**
     * UTM 좌표계에서 유클리드 거리 계산
     * 
     * @param easting1 첫 번째 지점의 UTM Easting 좌표 (동쪽, 미터)
     * @param northing1 첫 번째 지점의 UTM Northing 좌표 (북쪽, 미터)
     * @param easting2 두 번째 지점의 UTM Easting 좌표 (동쪽, 미터)
     * @param northing2 두 번째 지점의 UTM Northing 좌표 (북쪽, 미터)
     * @return 두 지점 간 거리 (미터)
     */
    static double euclideanDistanceUTM(double easting1, double northing1, 
                                       double easting2, double northing2) {
        double d_easting = easting2 - easting1;
        double d_northing = northing2 - northing1;
        return std::sqrt(d_easting * d_easting + d_northing * d_northing);
    }

    /**
     * UTM 좌표계에서 내 위치 기준으로 A의 반대 방향 distance(m) 떨어진 S 지점 계산
     * 
     * @param my_easting 내 위치 UTM Easting 좌표 (동쪽, 미터)
     * @param my_northing 내 위치 UTM Northing 좌표 (북쪽, 미터)
     * @param a_easting A 지점 UTM Easting 좌표 (동쪽, 미터)
     * @param a_northing A 지점 UTM Northing 좌표 (북쪽, 미터)
     * @param distance S와 내 위치 사이의 거리 (미터, 기본값 30m)
     * @return S 지점의 UTM 좌표
     */
    static UTMPoint calculateSPointUTM(double my_easting, double my_northing, 
                                       double a_easting, double a_northing, 
                                       double distance = 30.0) {
        // A에서 내 위치로의 방향 벡터
        double d_easting = my_easting - a_easting;
        double d_northing = my_northing - a_northing;
        
        // 벡터의 길이
        double length = std::sqrt(d_easting * d_easting + d_northing * d_northing);
        
        if (length == 0) {
            throw std::invalid_argument("내 위치와 A 지점이 동일합니다.");
        }
        
        // 단위 벡터 (A에서 내 위치 방향)
        double u_easting = d_easting / length;
        double u_northing = d_northing / length;
        
        // S 지점 계산 (내 위치에서 A의 반대 방향으로 distance만큼)
        // UTM 좌표계에서는 미터 단위이므로 직접 계산
        double s_easting = my_easting + u_easting * distance;
        double s_northing = my_northing + u_northing * distance;
        
        return UTMPoint(s_easting, s_northing);
    }
};

int main() {
    try {
        // 예제 UTM 좌표 (예: 한국 서울 지역 - UTM Zone 52N)
        // 실제 사용시에는 위도/경도를 UTM으로 변환한 값을 사용
        double my_easting = 323394.0;    // UTM Easting 좌표 (동쪽, 미터)
        double my_northing = 4161274.0;  // UTM Northing 좌표 (북쪽, 미터)
        double a_easting = 323450.0;     // A 지점 UTM Easting 좌표 (동쪽, 미터)
        double a_northing = 4161330.0;   // A 지점 UTM Northing 좌표 (북쪽, 미터)
        
        // S 지점 계산
        UTMStartPosition::UTMPoint s_point = 
            UTMStartPosition::calculateSPointUTM(my_easting, my_northing, 
                                                 a_easting, a_northing, 30.0);
        
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "==================================================" << std::endl;
        std::cout << "S 지점 계산 결과 (UTM 좌표계)" << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "내 위치: (E:" << my_easting << ", N:" << my_northing << ") m" << std::endl;
        std::cout << "A 지점:  (E:" << a_easting << ", N:" << a_northing << ") m" << std::endl;
        std::cout << "S 지점:  (E:" << s_point.easting << ", N:" << s_point.northing << ") m" << std::endl;
        std::cout << std::endl;
        
        // 거리 검증
        double dist_s_to_my = UTMStartPosition::euclideanDistanceUTM(
            s_point.easting, s_point.northing, my_easting, my_northing);
        double dist_my_to_a = UTMStartPosition::euclideanDistanceUTM(
            my_easting, my_northing, a_easting, a_northing);
        
        std::cout << std::setprecision(2);
        std::cout << "S → 내 위치 거리: " << dist_s_to_my << "m (목표: 30m)" << std::endl;
        std::cout << "내 위치 → A 거리: " << dist_my_to_a << "m" << std::endl;
        std::cout << "==================================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "오류: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
