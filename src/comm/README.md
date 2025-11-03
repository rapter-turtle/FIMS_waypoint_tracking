# Bada Comm Package

ROS 2 통신 패키지로, Bada 프로젝트에서 사용하는 다양한 통신 노드들을 포함합니다.

## 노드 설명

### bada_mavlink 노드

이 노드는 pymavlink 라이브러리를 사용하여 MAVLink 프로토콜로 Pixhawk 또는 다른 MAVLink 장치와 통신하고, 수신한 메시지를 ROS 2 토픽으로 발행합니다.

#### 지원하는 통신 방식
- **UDP**: 기본 설정은 IP `127.0.0.1`, 수신 포트 `15555`, 송신 IP `192.168.0.1`, 송신 포트 `15550` 입니다.
- **Serial**: 기본 설정은 장치 `/dev/ttyUSB0`, 통신 속도 `115200` 입니다.

#### 수신하는 MAVLink 메시지
- **HEARTBEAT**: 시스템 모드 정보
- **STATUS**: 시스템 상태 정보
- **GLOBAL_POSITION_INT**: GPS 위치 정보 (위도/경도/고도)
- **LOCAL_POSITION_NED**: 로컬 위치 및 속도 정보
- **ATTITUDE**: 자세 정보 (롤/피치/요)
- **ACTUATOR_OUTPUT_STATUS**: 액추에이터 출력 정보
- **MISSION_CURRENT**: 현재 실행 중인 미션 아이템
- **MISSION_ITEM_INT**: 미션 아이템의 좌표 정보

#### 발행하는 ROS 2 토픽
- **/bada/boat_mode** (Type: Mode): 보트의 현재 운행 모드
- **/bada/boat_position** (Type: GlobalPosition): 보트의 현재 GPS 위치
- **/bada/boat_speed** (Type: Speed): 보트의 현재 속도
- **/bada/boat_attitude** (Type: Attitude): 보트의 현재 자세
- **/bada/pixhawk_actuator** (Type: ActuatorOutputs): Pixhawk의 액추에이터 출력값
- **/bada/target_position** (Type: GlobalPosition): 목표 위치 (미션 아이템)

## 설치 방법

1. ROS 2 워크스페이스의 `src` 폴더에 패키지 클론
2. 의존성 설치
   ```bash
   sudo apt-get install python3-pip
   pip3 install pymavlink
   ```
3. 패키지 빌드
   ```bash
   cd ~/bada2_ws
   colcon build --packages-select comm
   source install/setup.bash
   ```

## 실행 방법

### 기본 설정으로 실행
```bash
ros2 launch comm bada_mavlink_launch.py
```

### UDP 설정 변경하여 실행
```bash
ros2 launch comm bada_mavlink_launch.py udp_local_ip:=192.168.1.100 udp_local_port:=14550
```

### Serial 연결로 실행
```bash
ros2 launch comm bada_mavlink_launch.py connection_type:=serial serial_device:=/dev/ttyACM0
```

## 파라미터 설명

| 파라미터 이름 | 기본값 | 설명 |
|--------------|------|------|
| connection_type | 'udp' | 연결 방식 ('udp' 또는 'serial') |
| udp_local_ip | '127.0.0.1' | UDP 로컬 IP 주소 |
| udp_local_port | 15555 | UDP 로컬 포트 |
| udp_remote_ip | '192.168.0.1' | UDP 원격 IP 주소 |
| udp_remote_port | 15550 | UDP 원격 포트 |
| serial_device | '/dev/ttyUSB0' | 시리얼 장치 경로 |
| serial_baudrate | 115200 | 시리얼 통신 속도 |
| update_rate | 10 | 메시지 발행 주기 (Hz) |


## plccomm.py
* flight_mode, armed 체크
   * armed면서 stabilized, manual, posctl
      * topic : /bada/px4_actuator 
   * armed면서 loiter, mission
      * topic : /bada/ros_actuator

* topic : /bada/px4_actuator , Float64MultiArray type을 subscribe한다.
* topic : /bada/ros_actuator , Float64MultiArray type을 subscribe한다.

topic :/bada/mode에서 Mode 타입을 subscribe한다. 
mode와 is_armed 변수에 할당한다.


-----
publish_estimated_state method를 생성하고 12개의 item을 가지는 Float64MultiArray를 생성하여 
mavlink에서 수신한 좌표계는 NED를 기반으로 하지만 item에 사용하는 것들은 모두 ENU를 사용한다. 

0, 1 : GLOBAL_POSITION_INT에서 위도, 경도를 가지고 UTM easting과 northing을 계산하며 단위는 m 이다.
2 : ATTITUDE msg에서 yaw를 ENU기준으로 값을 할당
3 : GLOBAL_POSITION_INT 에서 vx를 가져와서 m/s로 변환한다.
4 : GLOBAL_POSITION_INT 에서 vy를 가져와서 sway 로 변환하여 m/s로 변환한다.
5 : ATTITUDE에서 yaw rate을 ENU기준으로 변환하여 값을 할당
6, 7 : GPS_RAW_INT에서 위도, 경도를 가지고 UTM easting과 northing를 계산하며 단위는 m 이다. 
8 : ATTITUDE에서 yaw를 ENU 기준으로 할당
9 : GLOBAL_POSITION_INT 에서 vx와 vy를 가져와서 벡터 합으로 m/s로 변환한다.
10 : GPS_RAW_INT에서 vy sway로 기준으로 m/s 단위로 변환
11 : ATTITUDE에서 yaw를 ENU 기준으로 변환한다. 단위는 rad/s

## 조건
* plc write
   1. no
      * disarm
   2. px4
      * arm/manual-stabilize-posctl
   3. ros
      * arm/mission-active

