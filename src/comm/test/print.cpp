#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// MAV_MODE_FLAG 비트 정의
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED    1
#define MAV_MODE_FLAG_TEST_ENABLED           2
#define MAV_MODE_FLAG_AUTO_ENABLED           4
#define MAV_MODE_FLAG_GUIDED_ENABLED         8
#define MAV_MODE_FLAG_STABILIZE_ENABLED     16
#define MAV_MODE_FLAG_HIL_ENABLED           32
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED  64
#define MAV_MODE_FLAG_SAFETY_ARMED         128

// PX4 Custom Mode 정의 (px4_custom_mode.h에서 가져옴)
// PX4의 비행 모드는 main_mode와 sub_mode로 구성됨
typedef union {
    struct {
        uint16_t reserved;
        uint8_t main_mode;
        uint8_t sub_mode;
    };
    uint32_t data;
    float data_float;
} px4_custom_mode_t;

// PX4 Main Modes
typedef enum {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE,
    PX4_CUSTOM_MAIN_MODE_SIMPLE    /* unused, but reserved for future use */
} PX4_CUSTOM_MAIN_MODE;

// PX4 Auto Sub Modes
typedef enum {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
} PX4_CUSTOM_SUB_MODE_AUTO;

// 비행 모드 열거형
typedef enum {
    FLIGHT_MODE_UNKNOWN = 0,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_POSITION,
    FLIGHT_MODE_MISSION,
    FLIGHT_MODE_GO_TO_LOCATION,
    FLIGHT_MODE_RTL,
    FLIGHT_MODE_LAND,
    FLIGHT_MODE_TAKEOFF,
    FLIGHT_MODE_OFFBOARD,
    FLIGHT_MODE_MANUAL,
    FLIGHT_MODE_ALTITUDE_CONTROL
} flight_mode_t;

/**
 * PX4 비행 모드 상태를 판단하는 함수
 * @param base_mode HEARTBEAT 메시지의 base_mode 필드
 * @param custom_mode HEARTBEAT 메시지의 custom_mode 필드
 * @return flight_mode_t 비행 모드 열거형 값
 */
flight_mode_t px4_get_flight_mode(uint8_t base_mode, uint32_t custom_mode) {
    // Armed 상태 확인
    bool is_armed = (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    
    // Custom mode가 활성화되지 않은 경우 base_mode만으로 판단
    if (!(base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
        if (base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) {
            return FLIGHT_MODE_STABILIZE;
        }
        if (base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) {
            return FLIGHT_MODE_GO_TO_LOCATION;
        }
        if (base_mode & MAV_MODE_FLAG_AUTO_ENABLED) {
            return FLIGHT_MODE_MISSION;
        }
        return FLIGHT_MODE_UNKNOWN;
    }
    
    // Custom mode 파싱
    px4_custom_mode_t px4_mode;
    px4_mode.data = custom_mode;
    
    switch (px4_mode.main_mode) {
        case PX4_CUSTOM_MAIN_MODE_MANUAL:
            return FLIGHT_MODE_MANUAL;
            
        case PX4_CUSTOM_MAIN_MODE_STABILIZED:
            return FLIGHT_MODE_STABILIZE;
            
        case PX4_CUSTOM_MAIN_MODE_ALTCTL:
            return FLIGHT_MODE_ALTITUDE_CONTROL;
            
        case PX4_CUSTOM_MAIN_MODE_POSCTL:
            return FLIGHT_MODE_POSITION;
            
        case PX4_CUSTOM_MAIN_MODE_AUTO:
            // Auto 모드의 sub-mode로 세부 판단
            switch (px4_mode.sub_mode) {
                case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
                    return FLIGHT_MODE_MISSION;
                case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                    return FLIGHT_MODE_GO_TO_LOCATION;
                case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                    return FLIGHT_MODE_RTL;
                case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                    return FLIGHT_MODE_LAND;
                case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                    return FLIGHT_MODE_TAKEOFF;
                default:
                    return FLIGHT_MODE_MISSION; // 기본적으로 AUTO는 MISSION으로 간주
            }
            
        case PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            return FLIGHT_MODE_OFFBOARD;
            
        default:
            return FLIGHT_MODE_UNKNOWN;
    }
}

/**
 * 비행 모드를 문자열로 변환하는 함수
 * @param mode flight_mode_t 열거형 값
 * @return 모드명 문자열
 */
const char* flight_mode_to_string(flight_mode_t mode) {
    switch (mode) {
        case FLIGHT_MODE_STABILIZE:      return "STABILIZE";
        case FLIGHT_MODE_POSITION:       return "POSITION";
        case FLIGHT_MODE_MISSION:        return "MISSION";
        case FLIGHT_MODE_GO_TO_LOCATION: return "GO_TO_LOCATION";
        case FLIGHT_MODE_RTL:            return "RTL";
        case FLIGHT_MODE_LAND:           return "LAND";
        case FLIGHT_MODE_TAKEOFF:        return "TAKEOFF";
        case FLIGHT_MODE_OFFBOARD:       return "OFFBOARD";
        case FLIGHT_MODE_MANUAL:         return "MANUAL";
        case FLIGHT_MODE_ALTITUDE_CONTROL: return "ALTITUDE_CONTROL";
        default:                         return "UNKNOWN";
    }
}

/**
 * 특정 모드인지 확인하는 함수들
 */
bool is_stabilize_mode(uint8_t base_mode, uint32_t custom_mode) {
    return px4_get_flight_mode(base_mode, custom_mode) == FLIGHT_MODE_STABILIZE;
}

bool is_position_mode(uint8_t base_mode, uint32_t custom_mode) {
    return px4_get_flight_mode(base_mode, custom_mode) == FLIGHT_MODE_POSITION;
}

bool is_mission_mode(uint8_t base_mode, uint32_t custom_mode) {
    return px4_get_flight_mode(base_mode, custom_mode) == FLIGHT_MODE_MISSION;
}

bool is_go_to_location_mode(uint8_t base_mode, uint32_t custom_mode) {
    flight_mode_t mode = px4_get_flight_mode(base_mode, custom_mode);
    return (mode == FLIGHT_MODE_GO_TO_LOCATION || mode == FLIGHT_MODE_OFFBOARD);
}

/**
 * Armed 상태 확인 함수
 */
bool is_armed(uint8_t base_mode) {
    return (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

/**
 * 종합적인 상태 정보 출력 함수
 */
void print_flight_status(uint8_t base_mode, uint32_t custom_mode) {
    flight_mode_t mode = px4_get_flight_mode(base_mode, custom_mode);
    bool armed = is_armed(base_mode);
    
    px4_custom_mode_t px4_mode;
    px4_mode.data = custom_mode;
    
    printf("=== PX4 Flight Status ===\n");
    printf("Base Mode: 0x%02X (%d)\n", base_mode, base_mode);
    printf("Custom Mode: 0x%08X (%u)\n", custom_mode, custom_mode);
    printf("  - Main Mode: %d\n", px4_mode.main_mode);
    printf("  - Sub Mode: %d\n", px4_mode.sub_mode);
    printf("Flight Mode: %s\n", flight_mode_to_string(mode));
    printf("Armed: %s\n", armed ? "YES" : "NO");
    
    // 요청된 모드들 체크
    printf("\n=== Mode Checks ===\n");
    printf("Stabilize Mode: %s\n", is_stabilize_mode(base_mode, custom_mode) ? "YES" : "NO");
    printf("Position Mode: %s\n", is_position_mode(base_mode, custom_mode) ? "YES" : "NO");
    printf("Mission Mode: %s\n", is_mission_mode(base_mode, custom_mode) ? "YES" : "NO");
    printf("Go to Location Mode: %s\n", is_go_to_location_mode(base_mode, custom_mode) ? "YES" : "NO");
}

// 사용 예시
int main() {
    // 예시 테스트 케이스들
    printf("=== PX4 Flight Mode Detection Test ===\n\n");
    
    // 테스트 케이스 1: Stabilized mode
    printf("Test 1: Stabilized Mode\n");
    print_flight_status(209, 0x070000); // Armed, Stabilized
    printf("\n");
    
    // 테스트 케이스 2: Position mode  
    printf("Test 2: Position Mode\n");
    print_flight_status(209, 0x030000); // Armed, Position Control
    printf("\n");
    
    // 테스트 케이스 3: Mission mode
    printf("Test 3: Mission Mode\n");
    print_flight_status(217, 0x040400); // Armed, Auto Mission
    printf("\n");
    
    // 테스트 케이스 4: Offboard mode (Go to Location)
    printf("Test 4: Offboard Mode (Go to Location)\n");
    print_flight_status(217, 0x060000); // Armed, Offboard
    printf("\n");
    
    return 0;
}