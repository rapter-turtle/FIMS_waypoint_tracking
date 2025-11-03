# bada2_ws
* ROS 2 humble용 ROS workspace
* 아래와 같은 package로 구성되며 

## dependency
```bash
sudo apt update
sudo apt install libgeographic-dev
pip install utm
pip install pyserial
pip install pymavlink


sudo apt update
sudo apt upgrade -y
sudo apt install -y python3 python3-pip python3-dev git
pip3 install "numpy<2"
pip install --upgrade pip
pip install MAVProxy

# modem manager 삭제
sudo apt remove --purge modemmanager
sudo apt autoremove

sudo usermod -a -G dialout $USER
sudo reboot


mavproxy.py --master=/dev/ttyUSB0 --baudrate 115200 \
             --out=udp:127.0.0.1:15678 \
             --out=udp:192.168.101.10:14550


// /dev/ttyUSB0 권한 설정
// 서비스 등록
/etc/systemd/system/bada-launch.service

```

* Ubuntu 24.04
```
sudo apt install libgeographiclib-dev
pipx install pymavlink
pip3 install pymavlink --break-system-packages
pipx install pyserial
pip install utm
```


## IP
* IP : 192.168.2.x  (x는 1자리수로 4 이상)
* Mask ; 255.255.255.0
* DNS : 0.0.0.1
* Gateway : 192.168.2.1

## RC
### 페어링
   * '+' 버튼 길게
   * MDL-SEL 선택
   * RX : T-FHSS AIR 선택된 상태 확인
   * LINK 선택
   * 버튼 길게 3초 이상
      * Beep 음
   * 수신기 Off -> On
      * 녹색 LED (성공)

### Channel 설정
* '+' 버튼 길게
* AUX-CHAN 선택
   * CH5 : SwC
   * CH6 : SwD
   * CH7 : SwA
   * CH8 : SwB
   * CH9 : SwE
   * CH10 : NULL

* [Futaba T10J 조정기 메뉴얼](http://www.xcopter.com/web/manual/radio/10j-manual.pdf)

## topic list
* actuator
   * /bada/px4_actuator
   * /bada/ros_actuator

* status
   * /bada/mode
   * /bada/attitude
   * /bada/gps
   * /bada/position
   * /bada/target_position

* estimation
   * /bada/estimated_state

* mission
   * /bada/mission_current
      * seq : 
      * total : 
      * state : MISSION_STATE_ACTIVE , MISSION_STATE_COMPLETE
   * /bada/mission_item_reached
   * /bada/waypoints
   * /bada/control_parameters
   * /bada/desired_velocity

---------

## rule
* topic name
   * "/bada/" prefix
   * 
## packages
* comm
* controller
* bada_msg

## bada_msg
* Mode (수동, 자동, ...)
```
int8 mode
```

* ActuatorOuts
```
uint32 active
float32[32] actuator
```

* ControlParameters
```
float64 acceptance_radius  
float64 kp
float64 kd 
float64 desired_velocity 
float64 max_steer
float64 max_thrust_diff
float64 max_steer_diff
float64 max_thrust
float64 kup 
float64 kud
float64 kui
```

* PlcStatus
```
int32 auto_control_status
int32 emergency_stop_status
int32 engine_rpm_status
int32 clutch_status
int32 steering_angle_status 
int32 trim_angle_status
int32 engine_running_status 
int32 bow_thruster_power_status 
int32 bow_thruster_rev_status 
```

* Attitude
```
float32 roll
float32 pitch
float32 yaw
```

* GlobalPosition
```
float64 lat
float64 lon
float64 alt
```

* Speed 
```
float32 vx
float32 vy
float32 vz
```



## comm package
* 모든 node들은 python node들고 구성되어 있다.
* bada_mavlink node
   * pymavlink는 UDP와 serial 통신을 지원하며 default는 UDP이다. 
      * UDP는 IP:127.0.0.1 Port : 15555로 수신하고, 송신은 IP:192.168.0.1 Port : 15550
      * Serial 통신은 '/dev/ttyUSB0' port로 baud rate은 115200 으로 통신한다.
   * pymavlink receive msg (udp 통신과 )
      * Heartbeat (Mode)
      * Status 
      * GLOBAL_POSITION_INT (위도/경도/고도를 )
      * LOCAL_POSITION_NED
      * ATTITUDE (or ATTITUDE_QUATERNION)
      * ACTUATOR_OUTPUT_STATUS
      * **MISSION_CURRENT**: 현재 실행 중인 미션 아이템
      - **MISSION_ITEM_INT**: 미션 아이템의 좌표 정보
   * pub
      * topic : "/bada/boat_mode", msg type : Mode
         * Heartbeat 메시지를 수신해서 custom_mode 값을 가져와서 사용
      * topic : "/bada/boat_position" ,msg type : GlobalPosition
         * GLOBAL_POSITION_INT 메시지에서 위도,경도,고도를 가져와서 사용
      * topic : "/bada/boat_speed" msg type :  Speed 
         * LOCAL_POSITION_NED 메시지에서 vx, vy, vz 를 가져와서 사용
      * topic : "/bada/boat_attitude" msg type : Attitude
         * ATTITUDE 메시지에서 roll, pitch, yaw를 가져와서 사용
      * topic : "/bada/pixhawk_actuator", msg type : ActuatorOuts
         * ACTUATOR_OUTPUT_STATUS를 가져와서 사용
      * topic : "/bada/target_position" ,msg type : GlobalPosition
         * MISSION_ITEM_INT에서 좌표 정보를 가져와서 사용
   * sub
      * ?
* plc node
   * sub
      * topic : "/bada/boat_mode", msg type : Mode 
      * topic : "/bada/pixhawk_actuator", msg type : ActuatorOuts
      * topic : "/badasg_type == 'GLOBAL_POSITION_INT'/ros_actuator" , msg_type : ActuatorOuts
   * write to PLC
      * UDP write를 구현한다.

## 기존 controller (코드 확인하기)
* ekf_px4
   * ekf_filter_px4.cpp
* actuator publisher
   * thrust_check.py
* thrust
   * aura_wpt_model_carrot_pid_thrust.cpp

## controller package
* c++ node들과 python node들로 구성되어 있다.
* bada_controller node는 
   * pub
      * topic : "/bada/ros_acturator" , msg_type : ActuatorOuts
   * sub
      * topic : params , msg_type : 
* bada_estimator node
   * 20 Hz로 동작하며 
   * pub
      * 20Hz로 동작
      * u, v, w와 attitude를 수신한 경우
      * topic : estimator
   * sub
      * 
      * topic : 

