#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "aura_msg/msg/actuator_outputs.hpp"
#include "aura_msg/msg/waypoint.hpp"
#include "aura_msg/msg/parameter.hpp"
#include "bada_msg/msg/mission_current.hpp"
#include "bada_msg/msg/mission_item_reached.hpp"
#include "bada_msg/msg/mode.hpp"
#include "bada_msg/msg/global_position.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
/////////////////////////////////////////////////////////////////////////////////////////////////    -> 각 라인 다시 확인
// 1. desired_velocity : /bada/speed 에서 desired velocity(get_vel 변수에 저장)가 잘 업데이트 되는지 확인 필요.  (line 231)

// 2. calculateSPoint : Latlon 으로 비교하게 되면 현 지점과 wpt 의 위치가 같게 나오는 경우가 있음. -> utm으로 변환 후 비교 추천 (line 112)

// 3. Thrust input : 현재 thrust는 desired velocity(get_vel 변수에 저장)에 비례하여 적용됨. px4, 혹은 사람이 직접 desired velocity를 /bada/speed로 주게 되면, 그 속도에 비례한 일정한 값의 throttle 값이 걸리게 될 것임 (line 581)
//                            -> 이때, 명령어로 주는 속도 값이 실제로 나오는 속도값과 다를 수 있음. 따라서 실험을 통해 실제 속도에 맞는 desired velocity값을 표로 작성하여 사용하는 것을 추천 

// 4. Gain을 튜닝할 때에는 rqt에서 "/bada/control_parameters"의 토픽을 사용하여 gain tuinning을 하면 됨 -> "update_gains_based_on_velocity"에서 튜닝 모드로 전환 후 튜닝 (line 361)

// 5. 현재 코드에서, "throttle의 변화량"에 제한을 걸어둠. 따라서 0에서 시작하여 천천히 throttle이 증가하게됨 (max_thrust_diff 변수 사용하여 조정). 따라서 실제로 제어 알고리즘을 시작 할 때에 다른 조작을 가하지 않아도 됨. (line 149) 
//    !! 그러나 미션이 중단되었을 때, 다시 throttle 값을 0으로 초기화 시켜주어야함!! -> 그러지 않으면, 첫번째 미션을 중단하고 그 다음 미션을 시작할 때 첫번째 미션의 마지막 throttle 입력이 그대로 들어가 급발진 하게됨.

// 6. 현 wpt에서 다음 wpt로 넘어가게 하는 acceptance radius는 px4 코드에서 3~40m로 설정해 주는 것이 안전합니다
/////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
constexpr const T& clamp(const T& value, const T& low, const T& high)
{
    return (value < low) ? low : (value > high ? high : value);
}

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

    ////////////////////////////////////////////////////////// 두번쨰 사항 //////////////////////////////////////////////////////////
    static Point calculateSPoint(double myLat, double myLon, 
                                double aLat, double aLon, 
                                double distance = 30.0) {
        // A에서 내 위치로의 방향 벡터
        double dx = (myLon - aLon);
        double dy = (myLat - aLat);
        
        // 벡터의 길이
        double length = std::sqrt(dx * dx + dy * dy);
        // cout<<"dx : " << dx <<" dy : " << dy << " length : "<<length << endl;
        
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
    ////////////////////////////////////////////////////////// 두번쨰 사항 //////////////////////////////////////////////////////////

};


class ActuatorPublisher : public rclcpp::Node
{
public:
    ////////////////////////////////////////////////////////// 다섯번쨰 사항 //////////////////////////////////////////////////////////
    ActuatorPublisher() 
        : Node("actuator_publisher"),
          k(0), start_n(0.0), x(0.0), y(0.0), u(0.0), v(0.0), r(0.0), Xu(0.10531), LLOS(0.0), psi(0.0), received_(false), ll_thrust(0.0), delta(50.0),
          acceptance_radius(8.0), Kp(300.0), Kd(2000.0), Kup(0.2), Kud(0.0), Kui(0.05), max_I(1), get_vel(0.0), n(0.0),
          desired_velocity(0.0), max_steer(150.0), max_thrust(48), max_thrust_diff(0.01), max_steer_diff(30.0), before_proposed_thrust(0.0),  //-- 더 빠르게 증가시키고 싶으면 max_thrust_diff 증가
          current_mission_index(0), total_mission_number(0), current_reached_index(0), mission_started(false),
          current_mode(0), current_armed(0)
    {
    ////////////////////////////////////////////////////////// 다섯번쨰 사항 //////////////////////////////////////////////////////////
        // RCLCPP_INFO(this->get_logger(), "Initializing ActuatorPublisher...");

        // QoS 설정 (bada_mavlink.py와 동일)
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // Publishers
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bada/ros_actuator", 10);
        utm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ship/utm", 10);

        // Subscribers
        subscriber_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/bada/estimated_state", qos_profile, std::bind(&ActuatorPublisher::state_callback, this, std::placeholders::_1));

        subscriber_desired_velocity_ = this->create_subscription<std_msgs::msg::Float64>(
            "/bada/speed", qos_profile, std::bind(&ActuatorPublisher::velocity_callback, this, std::placeholders::_1));

        subscriber_waypoints_ = this->create_subscription<aura_msg::msg::Waypoint>(
            "/bada/waypoints", qos_profile,
            std::bind(&ActuatorPublisher::waypoints_callback, this, std::placeholders::_1));

        subscriber_params_ = this->create_subscription<aura_msg::msg::Parameter>(
            "/bada/control_parameters", 10,
            std::bind(&ActuatorPublisher::parameters_callback, this, std::placeholders::_1));

        subscriber_mission_current_ = this->create_subscription<bada_msg::msg::MissionCurrent>(
            "/bada/mission_current", qos_profile,
            std::bind(&ActuatorPublisher::mission_current_callback, this, std::placeholders::_1));

        subscriber_mission_item_reached_ = this->create_subscription<bada_msg::msg::MissionItemReached>(
            "/bada/mission_item_reached", qos_profile,
            std::bind(&ActuatorPublisher::mission_item_reached_callback, this, std::placeholders::_1));

        subscriber_mode_ = this->create_subscription<bada_msg::msg::Mode>(
            "/bada/mode", qos_profile,
            std::bind(&ActuatorPublisher::mode_callback, this, std::placeholders::_1));

        subscriber_position_ = this->create_subscription<bada_msg::msg::GlobalPosition>(
            "/bada/position", qos_profile,
            std::bind(&ActuatorPublisher::position_callback, this, std::placeholders::_1));

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(333),
            std::bind(&ActuatorPublisher::timer_callback, this));

        diff_thrust_before = 0.0;
        before_error_angle = 0.0;
        last_steering = 0.0;
        last_thrust = 0.0;
        before_velocity_e = 0.0;
        I_thrust = 0.0;
    }

private:

    void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Extract yaw (psi) from quaternion
        x = msg->data[0];
        y = msg->data[1];
        psi = -msg->data[2]-0.5*3.141592;
        u = msg->data[3];
        v = msg->data[4];
        r = msg->data[5];
        
        gps_received_ = true;
        publishUtmCoordinates();

        imu_received_ = true;
    }
    ////////////////////////////////////////////////////////// 첫번째 사항 //////////////////////////////////////////////////////////
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Extract yaw (psi) from quaternion
        // get_vel = msg->data/1.94384;
        
    }    
    ////////////////////////////////////////////////////////// 첫번째 사항 //////////////////////////////////////////////////////////


    void waypoints_callback(const aura_msg::msg::Waypoint::SharedPtr msg)
    {
        waypoints.clear();
        carrot_waypoints.clear();
        mission_started = false;

        // Store the last received waypoint message
        last_waypoint_message = *msg;

        // RCLCPP_INFO(this->get_logger(), "Received waypoints message");

        if (msg->x_lat.size() == msg->y_long.size())
        {
            for (size_t i = 0; i < msg->x_lat.size(); i++)
            {
                double utm_easting, utm_northing;
                int zone;
                bool northp;

                // Convert lat/lon to UTM/UPS
                GeographicLib::UTMUPS::Forward(
                    msg->x_lat[i], msg->y_long[i], zone, northp, utm_easting, utm_northing
                );
                waypoints.emplace_back(utm_easting, utm_northing);
                RCLCPP_ERROR(this->get_logger(), "Waypoints Received!!!");

            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Mismatch in x_lat and y_long sizes. Ignoring waypoints.");
        }
    }

    void parameters_callback(const aura_msg::msg::Parameter::SharedPtr msg)
    {
        
        double vel = msg->desired_velocity/1.94384;
        int index = static_cast<int>(vel);

        acceptance_radius = msg->acceptance_radius;
        Kp_schedule[index] = msg->kp;
        Kd_schedule[index] = msg->kd;
        
        max_steer_schedule[index] = msg->max_steer;
        max_steer_diff_schedule[index] = msg->max_steer_diff;
        max_thrust_diff = msg->max_thrust_diff;
        Kup = msg->kup;
        Kud = msg->kud;
        Kui = msg->kui;
        max_thrust = msg->max_thrust;

    }

    void mission_current_callback(const bada_msg::msg::MissionCurrent::SharedPtr msg)
    {
        current_mission_index = msg->seq;
        total_mission_number = msg->total;
        
        RCLCPP_INFO(this->get_logger(), "Mission Current: index=%d, total=%d", 
                    current_mission_index, total_mission_number);
    }

    void mission_item_reached_callback(const bada_msg::msg::MissionItemReached::SharedPtr msg)
    {
        current_reached_index = msg->seq;
        
        RCLCPP_INFO(this->get_logger(), "Mission Item Reached: index=%d", current_reached_index);
    }

    void mode_callback(const bada_msg::msg::Mode::SharedPtr msg)
    {
        current_mode = msg->mode;
        current_armed = msg->is_armed;
        
        RCLCPP_INFO(this->get_logger(), "Mode: %d, Armed: %d", current_mode, current_armed);
    }

    void position_callback(const bada_msg::msg::GlobalPosition::SharedPtr msg)
    {
        // Store the last received position message
        last_position_message = *msg;
        
        // Store position data
        RCLCPP_INFO(this->get_logger(), "Position: lat=%.6f, lon=%.6f, alt=%.2f", 
                    msg->lat, msg->lon, msg->alt);
    }

    double convertSteeringToPwm(double steer) {
        // Map steering value to PWM based on the given formula
        const double pwm_center = 1500.0;

        if (steer >= 300) {
            // Steer above 300 maps directly to PWM 2000
            return 2000.0;
        } else if (steer >= 0 && steer < 300) {
            // Steer in the range [0, 300] maps linearly between PWM = 1500 and PWM = 2000
            return 1500.0 + (steer * 1.6667);
        } else if (steer >= -300 && steer < 0) {
            // Steer in the range [-300, 0] maps linearly between PWM = 1000 and PWM = 1500
            return 1500.0 + (steer * 1.6667);
        } else {//if (steer < -300) {
            // Steer below -300 maps directly to PWM 1000
            return 1000.0;
        }

    }

    // Convert thrust level to PWM signal
    double convertThrustToPwm(double thrust) {
        if (thrust <= 0) {
            return 1500; // Any value <= 0 thrust maps to PWM 1000
        } else {
            // Calculate PWM based on the thrust
            double pwm = 3.9 * thrust + 1550;
            // double pwm = 5.0 * thrust + 1500;
            return clamp(pwm, 1500.0, 2000.0); // Ensure PWM is within the bounds
        }
    }
    
    ////////////////////////////////////////////////////////// 네번쨰 사항 //////////////////////////////////////////////////////////
    void update_gains_based_on_velocity(double desired_velocity)
    {

        //////// !!!!!!!!!!!!!!! 튜닝 모드 !!!!!!!!!!!!!!!
        // Determine the index based on the integer value of desired_velocity
        // int index = static_cast<int>(desired_velocity);  // Integer part of the velocity value
        // // Ensure the index is within bounds (0 to 20)
        // index = std::min(index, static_cast<int>(Kp_schedule.size()) - 1);
        // // Set the gains based on the velocity
        // Kp = Kp_schedule[index];
        // Kd = Kd_schedule[index];
        // max_steer = max_steer_schedule[index];
        // max_steer_diff = max_steer_diff_schedule[index];
        

        //////// !!!!!!!!!!!!!!! 실제 사용 모드 !!!!!!!!!!!!!!!
        double du = desired_velocity*1.94384;
        // Kp, Kd, Kui
        if (du <= 10.0){
            Kp = 500.0;
            Kd = 1000.0;
            max_steer = 150.0;
            max_steer_diff = 7.0*3.5;
            Kui = 0.05;
            RCLCPP_INFO(this->get_logger(), "went in1");
        }
        else if (du > 10.0 && du <= 16.0 ){
            Kp = -60.0*(du - 10) + 500.0;
            Kd = 200.0;
            max_steer = 150.0;
            max_steer_diff = 15.0*3.5;
            Kui = 0.05;
            RCLCPP_INFO(this->get_logger(), "went in2");
        }
        else{
            Kp = 140.0;
            Kd = 200.0;
            max_steer = 150.0;
            max_steer_diff = 15.0*3.5;
            Kui = 0.05;
            RCLCPP_INFO(this->get_logger(), "went in3");
        }

        // Kup
        if (du <= 8.0){
            Kup = 0.5;
        }
        else if (du > 8.0 && du <= 13.0 ){
            Kup = -0.06*(du - 8.0) + 0.5;
        }
        else{
            Kup = 0.2;
        }        
        // RCLCPP_INFO(this->get_logger(), "Desired velocity : %.2f", du);

        // Optionally, log the updated values for debugging
        RCLCPP_INFO(this->get_logger(), "Kp=%.2f, Kd=%.2f, max_steer=%.2f, max_steer_diff=%.2f", Kp, Kd, max_steer, max_steer_diff);
        RCLCPP_INFO(this->get_logger(), "Kup=%.2f, Kui=%.2f", Kup, Kui);
    }

    ////////////////////////////////////////////////////////// 네번쨰 사항 //////////////////////////////////////////////////////////

    void timer_callback()
    {

        // Update gains based on the current desired velocity
        update_gains_based_on_velocity(desired_velocity);

        // waypoint.size()가 0이면 나가기
        if (waypoints.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No waypoints available.");
            return;
        }

        // 현재 arming이 disarm이며 나가기
        if (current_armed == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Currently disarmed.");
            return;
        }
        // mode가 FLIGHT_MODE_AUTO_MISSION가 아니면  나가기
        if (current_mode != bada_msg::msg::Mode::FLIGHT_MODE_AUTO_MISSION)
        {
            RCLCPP_WARN(this->get_logger(), "Currently not in MISSION mode.");
            return;
        }

        // mission total과 reached index가 같으면 나가기
        if (total_mission_number == current_reached_index + 1 && total_mission_number != 0)
        {
            RCLCPP_INFO(this->get_logger(), "Mission completed. All waypoints reached.");
            mission_started = false;
            return;
        }    // Stop the vehicle by setting thrust to zero

        // mission_started 가 false이면 : carrot_waypoints 생성하기 
        if (mission_started == false){
            // carrot_waypoints.clear();
            RCLCPP_INFO(this->get_logger(), "my_lat: %f, my_lon: %f, way_lat: %f, way_lon: %f", last_position_message.lat, last_position_message.lon, last_waypoint_message.x_lat[0], last_waypoint_message.y_long[0]);

            StartPosition::Point s = StartPosition::calculateSPoint(last_position_message.lat, last_position_message.lon, last_waypoint_message.x_lat[0], last_waypoint_message.y_long[0], 30.0);
            
            double utm_easting, utm_northing;
            int zone;
            bool northp;

            // Convert lat/lon to UTM/UPS
            GeographicLib::UTMUPS::Forward(
                s.lat, s.lon, zone, northp, utm_easting, utm_northing
            );

            carrot_waypoints.emplace_back(utm_easting, utm_northing);

            for (size_t i = 0; i < waypoints.size(); i++)
            {
                carrot_waypoints.emplace_back(waypoints[i].first, waypoints[i].second);
            }

            mission_started = true;
            RCLCPP_INFO(this->get_logger(), "Mission started. Carrot waypoints initialized.");
        }

        // carrot_waypoints란 ? waypoints와 같은 타입으로 waypoints의 size보다 1 더 크다. 
        // carrot_waypoints의 [0] 은 현재 위치에서 30m 뒷지점을 구하는 알고리즘을 이용해서 채우고 1부터는 기존 waypoints[0]부터 값을 복사해서 넣는다. 


        // Proceed with the rest of your existing control logic
        // if (k >= waypoints.size())
        // {
        //     RCLCPP_INFO(this->get_logger(), "All waypoints reached. Resetting to first waypoint.");
        //     k = 0;
        // }

        k = current_mission_index;

        if(k <= total_mission_number-1) // if (total_mission_number-1 == current_reached_index)
        {
            // if (start_n == 0.0){
            //     start_n = 1;
                
            //     // Start from the first waypoint (index 0)
            //     k = 0;
            //     RCLCPP_INFO(this->get_logger(), "Starting from first waypoint (index 0)");
            // }

            auto target_wpt = carrot_waypoints[k];
            double dx = 0.0;
            double dy = 0.0;
            double distance_to_wpt = 0.0;

            double x1 = 0.0;
            double y1 = 0.0;
            double x2 = 0.0;
            double y2 = 0.0;
            double wpt_angle = 0.0;
            double f = 0.0;
            double xo = 0.0;
            double yo = 0.0;

            
            x1 = carrot_waypoints[k].first;
            y1 = carrot_waypoints[k].second;
            x2 = carrot_waypoints[k+1].first;
            y2 = carrot_waypoints[k+1].second;
        

            wpt_angle = std::atan2(y2-y1, x2-x1);
            f = (y2-y1)/(x2-x1);

            xo = (f*f*x1 - f*y1 + x + f*y)/(f*f + 1);
            yo = f*(xo - x1) + y1;
            
            dx = xo + delta*std::cos(wpt_angle) - x;
            dy = yo + delta*std::sin(wpt_angle) - y;

            distance_to_wpt = std::sqrt(dx * dx + dy * dy);
            double wpt_end_distance = std::sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2));
            delta = 50.0;///////////////////////////////////////////////////////////////////// safe area
            if (wpt_end_distance <= delta){
                // k = k + 1;
            }

            if (psi > M_PI)
                psi -= 2 * M_PI;
            else if (psi < -M_PI)
                psi += 2 * M_PI;

            // Line of Sight (LOS) angle
            double LOS = std::atan2(dy, dx) - psi;
            if (LOS > M_PI)
                LOS -= 2 * M_PI;
            else if (LOS < -M_PI)
                LOS += 2 * M_PI;
            LLOS = LOS;

            double dt = 0.33333;


            double error_angle = LOS;
            double steer_input = -Kp * error_angle - Kd * (error_angle - before_error_angle)*0.1/dt;
            steer_input = clamp(steer_input, -max_steer, max_steer);
            before_error_angle = error_angle;

            // get_vel =1.5;
            desired_velocity = get_vel*(1 - 0.5*clamp((steer_input*steer_input)/150000, 0.0, 0.06));  //// /data/speed에서 받은 get_vel값을 이용하여 desired velocity 업데이트. 
            double velocity_e = 0.0;
            velocity_e = u - desired_velocity;
            double proposed_thrust = 0.0;
            
            // set_thrust = (0.10531*desired_velocity + 0.0181*desired_velocity*sqrt(desired_velocity*desired_velocity));

            // Thrust Regulation
            double safe_area = 1.0;    ///////////////////////////////////////////////////////////////////// safe area
            // double waypoint_start_distance = std::sqrt((xo - x1)*(xo - x1) + (yo - y1)*(yo - y1));
            double waypoint_start_distance = std::sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
            // //Decrease mode 

            
            ////////////////////////////////////////////////////////// 세번째 사항 //////////////////////////////////////////////////////////
            if (waypoint_start_distance <= safe_area){ 
                proposed_thrust = before_proposed_thrust;   
                RCLCPP_INFO(this->get_logger(), "##### proposed thrust ######");
            }
            else{
                proposed_thrust = (0.21*desired_velocity);    ////<-- 여기서 desired_velocity 값에 비례하여 thrust가 들어감 (line 556 확인)
                before_proposed_thrust = proposed_thrust;
            }
            ////////////////////////////////////////////////////////// 세번째 사항 //////////////////////////////////////////////////////////

            before_velocity_e = velocity_e;
 
            // Apply rate limiting
            double steer_change = steer_input - last_steering;
            steer_change = clamp(steer_change, -max_steer_diff, max_steer_diff);
            double steer = last_steering + steer_change;

            double thrust_change = proposed_thrust - last_thrust;
            thrust_change = clamp(thrust_change, -max_thrust_diff, max_thrust_diff);
            
            double thrust = 0.0;
            if (waypoint_start_distance <= safe_area){
                thrust = ll_thrust;//1.0*(1-0.1*(1 - std::exp(-0.1*nn)))*ll_thrust;
                last_thrust = thrust;
            }
            else{
                thrust = last_thrust + thrust_change;
                last_thrust = thrust;
                ll_thrust = thrust;
                nn = 0;
            }            
            
            // double thrust = last_thrust + thrust_change;
            // double remap_thrust = thrust/(0.00058466*std::cos(0.0040635*steer));
            double remap_thrust = thrust/(0.00058466);
            if (remap_thrust <= 0)
            {
                remap_thrust = -sqrt(-remap_thrust);
            }
            else
            { 
                remap_thrust = sqrt(remap_thrust);
            }

            // Store the last commands
            last_steering = steer;
            last_thrust = thrust;
            

            remap_thrust = clamp(remap_thrust, 0.0, max_thrust);
            // Convert to PWM signals
            double pwm_steer = convertSteeringToPwm(steer);
            double pwm_thrust = convertThrustToPwm(remap_thrust);

            std_msgs::msg::Float64MultiArray actuator_msg;
            actuator_msg.data.push_back(pwm_steer);
            actuator_msg.data.push_back(pwm_thrust);
            actuator_msg.data.push_back(LLOS*180/3.141592);
            actuator_msg.data.push_back(0.0);

            // RCLCPP_INFO(this->get_logger(), "check=%.2f", clamp((steer_input*steer_input)/150000, 0.0, 0.3));
            // RCLCPP_INFO(this->get_logger(), "WPT #=%d, distance = %2.f", k, wpt_end_distance);
            RCLCPP_INFO(this->get_logger(), "steer=%.2f, thrust=%.2f", steer, remap_thrust);
            // RCLCPP_INFO(this->get_logger(), "LOS=%.2f", LOS * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "desired u =%.2f knots, u=%.2f m/s, %.2f knots", desired_velocity* 1.94384,u , u * 1.94384);
            RCLCPP_INFO(this->get_logger(), "Kp=%.2f, Kd=%.2f, max_steer=%.2f, max_steer_diff=%.2f",Kp, Kd, max_steer, max_steer_diff);
            RCLCPP_INFO(this->get_logger(), "I thrust=%.2f", I_thrust);

            publisher_->publish(actuator_msg);

            // Check if waypoint reached
            // if (distance_to_wpt < acceptance_radius)
            // {
            //     k++;
            //     RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached. Moving to next waypoint.", k - 1);
            // }

            n = n + 1;
            nn = nn + 1;
        }
    }


    void publishUtmCoordinates()
    {
        std_msgs::msg::Float64MultiArray utm_msg;
        utm_msg.data = {x, y, LLOS*180/M_PI};
        utm_publisher_->publish(utm_msg);
        // RCLCPP_INFO(this->get_logger(), "Published UTM Coordinates: x=%.2f, y=%.2f", x, y);
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    // rclcpp::Publisher<aura_msg::msg::ActuatorOutputs>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr utm_publisher_;



    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_;
    rclcpp::Subscription<aura_msg::msg::Waypoint>::SharedPtr subscriber_waypoints_;
    rclcpp::Subscription<aura_msg::msg::Parameter>::SharedPtr subscriber_params_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_desired_velocity_;
    rclcpp::Subscription<bada_msg::msg::MissionCurrent>::SharedPtr subscriber_mission_current_;
    rclcpp::Subscription<bada_msg::msg::MissionItemReached>::SharedPtr subscriber_mission_item_reached_;
    rclcpp::Subscription<bada_msg::msg::Mode>::SharedPtr subscriber_mode_;
    rclcpp::Subscription<bada_msg::msg::GlobalPosition>::SharedPtr subscriber_position_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;

    bool imu_received_ = false;
    bool gps_received_ = false;


    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> Kp_schedule = {
        300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 100.0, 100.0, 
        100.0, 400.0, 400.0, 400.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 
        200.0
    };

    std::vector<double> Kd_schedule = {
        2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 200.0, 
        200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 
        200.0
    };

    std::vector<double> max_steer_schedule = {
        150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 
        150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 
        150.0
    };

    std::vector<double> max_steer_diff_schedule = {
        7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 
        7.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 
        15.0
    };

    // Data variables
    size_t k;
    double x, y, psi, u, v, r, LLOS, n, nn, delta, start_n;
    std::vector<std::pair<double, double>> waypoints;
    std::vector<std::pair<double, double>> carrot_waypoints;
    double acceptance_radius, Kp, Kd, Kup, Kud, Kui, Xu, desired_velocity, max_thrust, max_steer, diff_thrust_before, before_error_angle, max_thrust_diff, max_steer_diff, last_steering, last_thrust, before_velocity_e, I_thrust, max_I, before_proposed_thrust;
    double get_vel, ll_thrust;
    bool received_;
    
    // Mission related variables
    int current_mission_index;
    int total_mission_number;
    int current_reached_index;
    bool mission_started;
    
    // Mode related variables
    int current_mode;
    int current_armed;
    
    // Waypoint message storage
    aura_msg::msg::Waypoint last_waypoint_message;
    
    // Position message storage
    bada_msg::msg::GlobalPosition last_position_message;
};

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ROS2 node...");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ActuatorPublisher>();
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down ROS2 node...");
    rclcpp::shutdown();
    return 0;
}