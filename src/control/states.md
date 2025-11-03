# EKF 추정 상태 데이터 필드 정의

## 개요
`ekf_filter.cpp`의 `publishState()` 함수에서 `/ekf/estimated_state` 토픽으로 퍼블리시되는 `std_msgs::msg::Float64MultiArray` 메시지의 12개 필드에 대한 상세 설명입니다.

## 좌표계 정의
- **ENU (East-North-Up)**: 월드 좌표계, 동쪽(+X), 북쪽(+Y), 위쪽(+Z)
- **Body**: 선체 좌표계, 전진(+X), 좌현(+Y), 상방(+Z)
- **UTM**: Universal Transverse Mercator 투영 좌표계

## 상태 필드 정의 (state_msg->data[0~11])

| Index | 필드명 | 의미 | 단위 | 좌표계 | 소스 | 비고 |
|-------|--------|------|------|--------|------|------|
| **[0]** | **X Position** | EKF 추정 선박 X 위치 (UTM Easting) | m | ENU/월드 (UTM) | `x_(0)` | GPS→UTM 변환으로 초기화/갱신 |
| **[1]** | **Y Position** | EKF 추정 선박 Y 위치 (UTM Northing) | m | ENU/월드 (UTM) | `x_(1)` | GPS→UTM 변환으로 초기화/갱신 |
| **[2]** | **Psi + 보정** | EKF 추정 요각 + 5° 보정값 | rad | ENU (동쪽=0°) | `x_(2) + 5π/180` | 고정 오프셋 +5° 적용 |
| **[3]** | **U (Surge)** | EKF 추정 전진 속도 | m/s | Body (선체) | `x_(3)` | NED의 X축과 동일 |
| **[4]** | **V (Sway)** | EKF 추정 좌현 속도 | m/s | Body (선체) | `x_(4)` | NED의 Y축과 동일 |
| **[5]** | **R (Yaw Rate)** | EKF 추정 요 각속도 | rad/s | Z축 회전 | `x_(5)` | NED에서는 -Z축 (부호 반전) |
| **[6]** | **GPS Easting** | GPS 원시 UTM Easting | m | ENU/월드 (UTM) | `z_gps[0]` | 위도/경도 역변환 가능 |
| **[7]** | **GPS Northing** | GPS 원시 UTM Northing | m | ENU/월드 (UTM) | `z_gps[1]` | 위도/경도 역변환 가능 |
| **[8]** | **IMU Heading** | IMU 요각 (저역통과 필터) | rad | ENU (동쪽=0°) | `z_imu[0]` | 스무딩 α=0.15, 주석≠실제값 |
| **[9]** | **Forward Speed** | 전진 속도 ([3]과 동일) | m/s | Body (선체) | `x_(3)` | 주석은 SOG지만 실제는 U만 |
| **[10]** | **GPS Vel Y** | GPS 좌현 속도 (선체좌표) | m/s | Body (선체) | `z_gps_vel[1]` | GPS 월드속도→선체좌표 변환 |
| **[11]** | **IMU Yaw Rate** | IMU Z축 각속도 원시값 | rad/s | Z축 회전 | `imu_yaw_rate` | `msg->angular_velocity.z` |

## 데이터 흐름도

```
GPS (/fix) → LatLongToUTM() → z_gps[0,1] → [6,7]
                            ↓
                         EKF 갱신 → x_[0,1] → [0,1]

IMU (/imu/data) → Quaternion→Euler → 스무딩 → z_imu[0] → [8]
                                   ↓
                              EKF 갱신 → x_[2] + 5° → [2]
                                      → x_[3,4,5] → [3,4,5]

GPS Vel (/fix_velocity) → 좌표변환 → z_gps_vel[1] → [10]

IMU 각속도 → imu_yaw_rate → [11]
           ↓
         EKF 갱신 → x_[5] → [5]
```

## UTM ↔ 위도/경도 변환

### 순변환 (위도/경도 → UTM)
현재 코드의 `LatLongToUTM()` 함수 사용 가능:
```cpp
double lat = 36.3664;  // 위도 (도)
double lon = 127.3647; // 경도 (도)
double easting, northing;
int zone;
bool northp;

LatLongToUTM(lat, lon, easting, northing, zone, northp);
```

### 역변환 (UTM → 위도/경도)
GeographicLib 라이브러리 사용 (권장):
```cpp
// 추가 구현 필요
void UTMToLatLong(double easting, double northing, int zone, bool northp,
                  double& lat, double& lon) {
    GeographicLib::UTMUPS::Reverse(zone, northp, easting, northing, lat, lon);
}

// 사용 예시
double lat, lon;
UTMToLatLong(z_gps[0], z_gps[1], 52, true, lat, lon);  // Zone 52N (한국)
```

## 주의사항

1. **좌표계 혼재**: 위치는 월드(UTM), 속도는 선체 좌표계 사용
2. **주석 불일치**: [8], [9] 필드의 주석과 실제 값이 다름
3. **단위 일관성**: 각도는 rad, 거리는 m, 속도는 m/s, 각속도는 rad/s
4. **보정값**: [2]에는 +5° 고정 오프셋 적용됨
5. **중복 데이터**: [3]과 [9]는 동일한 값 (전진 속도)
6. **UTM 변환**: 현재는 순변환만 구현, 역변환은 GeographicLib 추가 구현 필요

## 사용 예시

```cpp
// 상태 데이터 구독 예시
void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    double pos_x = msg->data[0];      // UTM X [m]
    double pos_y = msg->data[1];      // UTM Y [m] 
    double heading = msg->data[2];    // 요각 + 5° [rad]
    double surge = msg->data[3];      // 전진속도 [m/s]
    double sway = msg->data[4];       // 좌현속도 [m/s]
    double yaw_rate = msg->data[5];   // 요레이트 [rad/s]
    // ... 나머지 필드들
}
```

---
*생성일: 2025-09-29*  
*파일: `/ekf_filter.cpp` 기반*
