


#include <iostream>
#include <cmath>
#include <stdexcept>
#include <iomanip>

/**
 * StartPosition 클래스: 시작점 계산을 위한 클래스
 */
class StartPosition {
private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double EARTH_RADIUS = 6371000.0; // 지구 반지름 (미터)

public:
    /**
     * 좌표를 나타내는 구조체
     */
    struct Point {
        double lat;
        double lon;
        
        Point(double latitude, double longitude) 
            : lat(latitude), lon(longitude) {}
        
        Point() : lat(0.0), lon(0.0) {}
    };

    /**
     * 위도를 라디안으로 변환
     */
    static double toRadians(double degrees) {
        return degrees * PI / 180.0;
    }

    /**
     * 라디안을 위도로 변환
     */
    static double toDegrees(double radians) {
        return radians * 180.0 / PI;
    }

    /**
     * Haversine 공식을 사용한 두 지점 간 거리 계산
     * 
     * @param lat1 첫 번째 지점의 위도
     * @param lon1 첫 번째 지점의 경도
     * @param lat2 두 번째 지점의 위도
     * @param lon2 두 번째 지점의 경도
     * @return 두 지점 간 거리 (미터)
     */
    static double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
        double phi1 = toRadians(lat1);
        double phi2 = toRadians(lat2);
        double deltaPhi = toRadians(lat2 - lat1);
        double deltaLambda = toRadians(lon2 - lon1);
        
        double a = std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
                   std::cos(phi1) * std::cos(phi2) *
                   std::sin(deltaLambda / 2.0) * std::sin(deltaLambda / 2.0);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
        
        return EARTH_RADIUS * c;
    }

    /**
     * 내 위치 기준으로 A의 반대 방향으로 distance(m) 떨어진 S 지점 계산
     * 
     * @param myLat 내 위치 위도
     * @param myLon 내 위치 경도
     * @param aLat A 지점 위도
     * @param aLon A 지점 경도
     * @param distance S와 내 위치 사이의 거리 (미터, 기본값 30m)
     * @return S 지점의 좌표
     */
    static Point calculateSPoint(double myLat, double myLon, 
                                double aLat, double aLon, 
                                double distance = 30.0) {
        // A에서 내 위치로의 방향 벡터
        double dx = myLon - aLon;
        double dy = myLat - aLat;
        
        // 벡터의 길이
        double length = std::sqrt(dx * dx + dy * dy);
        
        if (length == 0.0) {
            throw std::invalid_argument("내 위치와 A 지점이 동일합니다.");
        }
        
        // 단위 벡터 (A에서 내 위치 방향)
        double ux = dx / length;
        double uy = dy / length;
        
        // 위도/경도를 미터로 변환하는 근사값
        double latPerMeter = 1.0 / 111000.0;
        double lonPerMeter = 1.0 / (111000.0 * std::cos(toRadians(myLat)));
        
        // S 지점 계산 (내 위치에서 A의 반대 방향으로 distance만큼)
        double sLat = myLat + uy * distance * latPerMeter;
        double sLon = myLon + ux * distance * lonPerMeter;
        
        return Point(sLat, sLon);
    }
};

/**
 * 메인 함수 - 사용 예제
 */
int main() {
    // 예제 좌표
    double myLat = 37.5660;
    double myLon = 126.9775;
    double aLat = 37.5665;
    double aLon = 126.9780;
    
    try {
        // S 지점 계산
        StartPosition::Point s = StartPosition::calculateSPoint(myLat, myLon, aLat, aLon, 30.0);
        
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "==================================================" << std::endl;
        std::cout << "S 지점 계산 결과" << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "내 위치: (" << myLat << ", " << myLon << ")" << std::endl;
        std::cout << "A 지점:  (" << aLat << ", " << aLon << ")" << std::endl;
        std::cout << "S 지점:  (" << s.lat << ", " << s.lon << ")" << std::endl;
        std::cout << std::endl;
        
        // 거리 검증
        double distSToMy = StartPosition::haversineDistance(s.lat, s.lon, myLat, myLon);
        double distMyToA = StartPosition::haversineDistance(myLat, myLon, aLat, aLon);
        
        std::cout << std::setprecision(2);
        std::cout << "S → 내 위치 거리: " << distSToMy << "m (목표: 30m)" << std::endl;
        std::cout << "내 위치 → A 거리: " << distMyToA << "m" << std::endl;
        std::cout << "==================================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "오류: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
/*
==================================================
컴파일 및 실행 방법 (C++)
==================================================

# 컴파일
g++ -std=c++11 main.cpp

# 실행
./a.out

==================================================
*/