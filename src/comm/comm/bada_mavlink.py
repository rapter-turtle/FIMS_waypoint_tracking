#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum, auto

from bada_msg.msg import Mode, GlobalPosition, Speed, Attitude, ActuatorOutputs, RcChannel
from aura_msg.msg import Waypoint

import time
import socket
import threading
import serial
import utm 

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink


# Mission Status Enum Definition
class MissionStatus(Enum):
    READY = 0     # Mission is ready to start
    DOING = 1     # Mission is currently in progress
    CANCEL = 2    # Mission has been cancelled
    COMPLETE = 3  # Mission has been completed successfully


# PX4 Custom Main Mode Definitions
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
PX4_CUSTOM_MAIN_MODE_SIMPLE = 9

# PX4 Custom Sub Mode Auto Definitions
PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

# PX4 Custom Sub Mode Position Control Definitions
PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL = 0
PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT = 1

class PX4FlightModeDecoder:
    """Decoder for PX4 custom flight modes from MAVLink heartbeat messages."""
    
    def __init__(self):
        """Initialize the flight mode decoder with mode mappings."""
        self.mode_names = {
            (PX4_CUSTOM_MAIN_MODE_MANUAL, None): "MANUAL",
            (PX4_CUSTOM_MAIN_MODE_STABILIZED, None): "STABILIZED",
            (PX4_CUSTOM_MAIN_MODE_ACRO, None): "ACRO",
            (PX4_CUSTOM_MAIN_MODE_RATTITUDE, None): "RATTITUDE",
            (PX4_CUSTOM_MAIN_MODE_ALTCTL, None): "ALTCTL",
            (PX4_CUSTOM_MAIN_MODE_OFFBOARD, None): "OFFBOARD",
            (PX4_CUSTOM_MAIN_MODE_SIMPLE, None): "SIMPLE",
            (PX4_CUSTOM_MAIN_MODE_POSCTL, PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL): "POSCTL",
            (PX4_CUSTOM_MAIN_MODE_POSCTL, PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT): "POSCTL_ORBIT",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_READY): "AUTO_READY",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF): "AUTO_TAKEOFF",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER): "AUTO_LOITER",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION): "AUTO_MISSION",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL): "AUTO_RTL",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND): "AUTO_LAND",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTGS): "AUTO_RTGS",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET): "AUTO_FOLLOW_TARGET",
            (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND): "AUTO_PRECLAND",
        }
    
    def decode_custom_mode(self, custom_mode):
        """
        Decode PX4 custom mode from MAVLink heartbeat custom_mode field.
        
        Args:
            custom_mode (int): The custom_mode field from MAVLink heartbeat message
            
        Returns:
            dict: Dictionary containing decoded mode information
        """
        # Extract main mode and sub mode from custom_mode
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF
        
        # For modes without sub-modes, use None as sub_mode key
        if main_mode not in [PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_MAIN_MODE_POSCTL]:
            lookup_key = (main_mode, None)
        else:
            lookup_key = (main_mode, sub_mode)
        
        mode_name = self.mode_names.get(lookup_key, "UNKNOWN")
        
        return {
            'mode_name': mode_name,
            'main_mode': main_mode,
            'sub_mode': sub_mode if main_mode in [PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_MAIN_MODE_POSCTL] else None,
            'custom_mode_raw': custom_mode
        }
    
    def is_manual_mode(self, custom_mode):
        """Check if the current mode is MANUAL."""
        decoded = self.decode_custom_mode(custom_mode)
        return decoded['mode_name'] == 'MANUAL'
    
    def is_stabilized_mode(self, custom_mode):
        """Check if the current mode is STABILIZED."""
        decoded = self.decode_custom_mode(custom_mode)
        return decoded['mode_name'] == 'STABILIZED'
    
    def is_position_mode(self, custom_mode):
        """Check if the current mode is POSCTL (Position Control)."""
        decoded = self.decode_custom_mode(custom_mode)
        return decoded['mode_name'] == 'POSCTL'
    
    def is_auto_mission_mode(self, custom_mode):
        """Check if the current mode is AUTO_MISSION."""
        decoded = self.decode_custom_mode(custom_mode)
        return decoded['mode_name'] == 'AUTO_MISSION'

def get_px4_flight_mode(custom_mode):
    """
    Extract flight mode information from PX4 custom_mode value.
    
    Args:
        custom_mode (int): The custom_mode value from MAVLink heartbeat message
        
    Returns:
        tuple: (mode_enum, mode_info)
            mode_enum (int): Mode enum value compatible with Mode.msg
            mode_info (dict): Flight mode information including specific mode checks
    """
    from bada_msg.msg import Mode
    
    decoder = PX4FlightModeDecoder()
    decoded_mode = decoder.decode_custom_mode(custom_mode)
    
    print(f"custom mode value : {custom_mode}, decoded: {decoded_mode}") 
    # Add specific mode checks
    mode_info = {
        **decoded_mode,
        'is_manual': decoder.is_manual_mode(custom_mode),
        'is_stabilized': decoder.is_stabilized_mode(custom_mode),
        'is_position': decoder.is_position_mode(custom_mode),
        'is_auto_mission': decoder.is_auto_mission_mode(custom_mode)
    }
    
    # Map mode_name to Mode.msg enum values
    mode_enum = Mode.FLIGHT_MODE_UNKNOWN  # Default
    
    if decoded_mode['mode_name'] == 'MANUAL':
        mode_enum = Mode.FLIGHT_MODE_MANUAL
    elif decoded_mode['mode_name'] == 'STABILIZED':
        mode_enum = Mode.FLIGHT_MODE_STABILIZE
    elif decoded_mode['mode_name'] == 'POSCTL':
        mode_enum = Mode.FLIGHT_MODE_POSITION
    elif decoded_mode['mode_name'] == 'AUTO_LOITER':
        mode_enum = Mode.FLIGHT_MODE_AUTO_LOITER
    elif decoded_mode['mode_name'] == 'AUTO_MISSION':
        mode_enum = Mode.FLIGHT_MODE_AUTO_MISSION
        
    return mode_enum, mode_info

def check_flight_modes(custom_mode):
    """
    Simple function to check specific flight modes from custom_mode value.
    
    Args:
        custom_mode (int): The custom_mode value from MAVLink heartbeat message
        
    Returns:
        dict: Boolean flags for manual, stabilized, position, and auto_mission modes
    """
    decoder = PX4FlightModeDecoder()
    
    return {
        'manual': decoder.is_manual_mode(custom_mode),
        'stabilized': decoder.is_stabilized_mode(custom_mode),
        'position': decoder.is_position_mode(custom_mode),
        'auto_mission': decoder.is_auto_mission_mode(custom_mode)
    }
class BadaMavlinkNode(Node):
    # MAV_MODE_FLAG 비트 정의
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_TEST_ENABLED = 2
    MAV_MODE_FLAG_AUTO_ENABLED = 4
    MAV_MODE_FLAG_GUIDED_ENABLED = 8
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16
    MAV_MODE_FLAG_HIL_ENABLED = 32
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
    MAV_MODE_FLAG_SAFETY_ARMED = 128

    # PX4 Main Modes
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8

    # PX4 Auto Sub Modes
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

    def __init__(self):
        super().__init__('bada_mavlink')
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # history=HistoryPolicy.BEST_EFFORT,
            depth=10
        )
        
        # 파라미터 선언
        self.declare_parameter('connection_type', 'udp') # self.declare_parameter('connection_type', 'serial')        # self.declare_parameter('connection_type', 'udp')
        self.declare_parameter('udp_local_ip', '127.0.0.1')
        self.declare_parameter('udp_local_port', 15678) # self.declare_parameter('udp_local_port', 15555)
        self.declare_parameter('udp_remote_ip', '192.168.0.1')
        self.declare_parameter('udp_remote_port', 15550)
        self.declare_parameter('serial_device', '/dev/ttyUSB0')# self.declare_parameter('serial_device', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('update_rate', 10)  # Hz
        
        # 파라미터 가져오기
        connection_type = self.get_parameter('connection_type').get_parameter_value().string_value
        self.udp_local_ip = self.get_parameter('udp_local_ip').get_parameter_value().string_value
        self.udp_local_port = self.get_parameter('udp_local_port').get_parameter_value().integer_value
        self.udp_remote_ip = self.get_parameter('udp_remote_ip').get_parameter_value().string_value
        self.udp_remote_port = self.get_parameter('udp_remote_port').get_parameter_value().integer_value
        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        
        # Publisher 생성
        self.boat_mode_pub = self.create_publisher(Mode, '/bada/mode', qos_profile)
        self.boat_position_pub = self.create_publisher(GlobalPosition, '/bada/position', qos_profile)
        self.boat_speed_pub = self.create_publisher(Speed, '/bada/speed', qos_profile)
        self.boat_attitude_pub = self.create_publisher(Attitude, '/bada/attitude', qos_profile)
        
        # Float64MultiArray 타입으로 변경하여 타입 일관성 유지
        from std_msgs.msg import Float64MultiArray
        self.pixhawk_actuator_pub = self.create_publisher(Float64MultiArray, '/bada/px4_actuator', qos_profile)
        
        self.target_position_pub = self.create_publisher(GlobalPosition, '/bada/target_position', qos_profile)
        self.waypoints_pub = self.create_publisher(Waypoint, '/bada/waypoints', 10)
        
        # desired_velocity 토픽 발행을 위한 퍼블리셔 추가
        from std_msgs.msg import Float64, Float64MultiArray
        self.desired_velocity_pub = self.create_publisher(Float64, '/bada/desired_velocity', qos_profile)
        
        # /ekf/estimated_state 토픽 발행을 위한 퍼블리셔 추가
        self.estimated_state_pub = self.create_publisher(Float64MultiArray, '/bada/estimated_state', qos_profile)
        
        # mission_current 및 mission_item_reached 토픽 발행을 위한 퍼블리셔 추가
        from bada_msg.msg import MissionCurrent, MissionItemReached
        self.mission_current_pub = self.create_publisher(MissionCurrent, '/bada/mission_current', qos_profile)
        self.mission_item_reached_pub = self.create_publisher(MissionItemReached, '/bada/mission_item_reached', qos_profile)
        
        # RC 채널 토픽 발행을 위한 퍼블리셔 추가
        self.rc_channels_pub = self.create_publisher(RcChannel, '/bada/rc_channels', qos_profile)
        
        # MAVLink 연결 설정
        try:
            if connection_type == 'serial':
                # Serial 연결 (dialect=common, mavlink_version=2.0)
                self.get_logger().info(f"MAVLink Serial Connection: {self.serial_device} at {self.serial_baudrate} baud")
                self.mavlink_connection = mavutil.mavlink_connection(
                    self.serial_device, 
                    baud=self.serial_baudrate,
                    dialect='common',
                    mavlink_version=2.0
                )
            else:
                # UDP 연결 (기본값)
                conn_string = f"udpin:{self.udp_local_ip}:{self.udp_local_port}"
                self.get_logger().info(f"MAVLink UDP Connection: {conn_string}, Remote: {self.udp_remote_ip}:{self.udp_remote_port}")
                self.mavlink_connection = mavutil.mavlink_connection(conn_string)
            
            self.get_logger().info("MAVLink connection established")
        except Exception as e:
            self.get_logger().error(f"Failed to establish MAVLink connection: {e}")
            return
        
        # MAVLink 메시지 수신 스레드 시작
        self.mavlink_thread = threading.Thread(target=self.receive_mavlink_messages)
        self.mavlink_thread.daemon = True
        self.mavlink_thread.start()
        
        # 미션 아이템 요청 스레드 시작
        self.mission_thread = threading.Thread(target=self.mission_request_thread)
        self.mission_thread.daemon = True
        self.mission_thread.start()
        
        self.get_logger().info("bada_mavlink node initialized")
        
        # 메시지 저장 변수
        self.last_heartbeat = None
        self.last_global_position_int = None
        self.last_local_position_ned = None
        self.last_attitude = None
        self.last_actuator_output_status = None
        self.last_mission_item_int = None
        self.last_position_target_local_ned = None  # POSITION_TARGET_LOCAL_NED 메시지 저장 변수 추가
        self.last_mission_current = None  # MISSION_CURRENT 메시지 저장 변수 추가
        self.last_mission_item_reached = None  # MISSION_ITEM_REACHED 메시지 저장 변수 추가
        self.last_gps_raw_int = None  # GPS_RAW_INT 메시지 저장 변수 추가
        
        # 미션 상태 변수 초기화
        self.mission_status = MissionStatus.READY  # 미션 상태를 READY로 초기화
        
        # 미션 아이템 관련 변수
        self.mission_items = []
        self.last_published_mission_items = []  # 이전에 발행한 미션 아이템 저장
        self.mission_count = 0
        self.mission_request_in_progress = False
        self.mission_sequence = 0
        self.mission_last_request_time = 0.0
        self.mission_speed_ms = 0.0  # 기본 속도 값
        
        # 비행 모드 열거형 정의 (Mode.msg에 정의된 것과 일치시켜야 함)
        self.FLIGHT_MODE = {
            "UNKNOWN": Mode.FLIGHT_MODE_UNKNOWN,
            "MANUAL": Mode.FLIGHT_MODE_MANUAL,
            "STABILIZE": Mode.FLIGHT_MODE_STABILIZE,
            "POSITION": Mode.FLIGHT_MODE_POSITION,
            "AUTO_LOITER": Mode.FLIGHT_MODE_AUTO_LOITER,
            "AUTO_MISSION": Mode.FLIGHT_MODE_AUTO_MISSION,
            # 아직 Mode.msg에 정의되지 않은 모드는 UNKNOWN으로 매핑
            "GUIDED": Mode.FLIGHT_MODE_UNKNOWN,
            "AUTO": Mode.FLIGHT_MODE_UNKNOWN,
            "RTL": Mode.FLIGHT_MODE_UNKNOWN,
            "LAND": Mode.FLIGHT_MODE_UNKNOWN
        }
    
    def receive_mavlink_messages(self):
        """
        MAVLink 메시지를 수신하는 스레드
        """
        self.get_logger().info("Starting MAVLink message receiving thread")
        
        while rclpy.ok():
            try:
                # MAVLink 메시지 수신 (non-blocking)
                msg = self.mavlink_connection.recv_match(blocking=False)
                if msg:
                    # 메시지 타입 확인
                    msg_type = msg.get_type()

                    if msg_type == 'HEARTBEAT':
                        self.last_heartbeat = msg
                        
                        # Heartbeat -> Mode 발행
                        # 비행 모드 분석 및 출력
                        custom_mode = msg.custom_mode
                        base_mode = msg.base_mode
                        if custom_mode == 0 and base_mode == 192:
                            pass
                        else:
                            self.get_logger().info(f"Heartbeat received: custom_mode={custom_mode}, base_mode={base_mode}")

                            # 비행 모드 판별 및 ROS 메시지 생성
                            from bada_msg.msg import Mode
                            mode_enum, mode_info = get_px4_flight_mode(custom_mode)
                            
                            # MAV_MODE_FLAG_SAFETY_ARMED 비트(7번째 비트, 0x80)를 확인하여 arming 상태 추출
                            MAV_MODE_FLAG_SAFETY_ARMED = 0x80
                            is_armed = (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
                            
                            # ROS 메시지 발행
                            mode_msg = Mode()
                            mode_msg.mode = mode_enum
                            mode_msg.is_armed = Mode.ARMED if is_armed else Mode.DISARMED
                            print('flight mode :', mode_msg.mode, 'armed :', 'Yes' if is_armed else 'No')
                            self.boat_mode_pub.publish(mode_msg)        # 비행모드 메시지 발행
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        self.last_global_position_int = msg
                        
                        # GLOBAL_POSITION_INT -> GlobalPosition 발행
                        position_msg = GlobalPosition()
                        position_msg.lat = msg.lat / 10000000.0  # MAVLink uses int32 * 10^7 for lat/lon
                        position_msg.lon = msg.lon / 10000000.0
                        position_msg.alt = msg.alt / 1000.0      # MAVLink uses mm for altitude
                        self.boat_position_pub.publish(position_msg)
                        
                        # GLOBAL_POSITION_INT 수신 시 estimated_state 발행
                        self.publish_estimated_state()
                    elif msg_type == 'LOCAL_POSITION_NED':
                        self.last_local_position_ned = msg
                        
                        # LOCAL_POSITION_NED -> Speed 발행
                        speed_msg = Speed()
                        speed_msg.vx = msg.vx
                        speed_msg.vy = msg.vy
                        speed_msg.vz = msg.vz
                        self.boat_speed_pub.publish(speed_msg)
                    elif msg_type == 'POSITION_TARGET_LOCAL_NED':
                        self.last_position_target_local_ned = msg
                        
                        # POSITION_TARGET_LOCAL_NED -> desired_velocity 발행
                        from std_msgs.msg import Float64
                        
                        # vx, vy를 이용하여 desired_velocity 계산
                        vx = msg.vx
                        vy = msg.vy
                        
                        # 속도 크기 계산 (vx와 vy로부터 합성 속도)
                        velocity_magnitude = (vx**2 + vy**2)**0.5
                        
                        # Float64 메시지 생성
                        desired_velocity_msg = Float64()
                        desired_velocity_msg.data = velocity_magnitude  # 속도의 크기만 저장
                        
                        # 토픽 발행
                        self.desired_velocity_pub.publish(desired_velocity_msg)
                        self.get_logger().debug(f"Published desired_velocity: {velocity_magnitude} (from vx={vx}, vy={vy})")
                    elif msg_type == 'ATTITUDE':
                        self.last_attitude = msg
                        
                        # ATTITUDE -> Attitude 발행
                        attitude_msg = Attitude()
                        attitude_msg.roll = msg.roll
                        attitude_msg.pitch = msg.pitch
                        attitude_msg.yaw = msg.yaw
                        self.boat_attitude_pub.publish(attitude_msg)
                    elif msg_type == 'SERVO_OUTPUT_RAW':
                        self.last_servo_output_raw = msg
                        from std_msgs.msg import Float64MultiArray
                        actuator_msg = Float64MultiArray()
                        actuator_msg.data = []
                        
                        # SERVO_OUTPUT_RAW 메시지의 servo1_raw ~ servo16_raw 필드 처리
                        servo_fields = [
                            'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw',
                            'servo5_raw', 'servo6_raw', 'servo7_raw', 'servo8_raw',
                            'servo9_raw', 'servo10_raw', 'servo11_raw', 'servo12_raw',
                            'servo13_raw', 'servo14_raw', 'servo15_raw', 'servo16_raw'
                        ]
                        
                        for field_name in servo_fields:
                            if hasattr(msg, field_name):
                                actuator_msg.data.append(float(getattr(msg, field_name)))
                        
                        self.pixhawk_actuator_pub.publish(actuator_msg)
                        # 수신한 서보 각 channel 값 로그 출력
                        # for i, value in enumerate(actuator_msg.data):
                        #     self.get_logger().info(f"SERVO_OUTPUT_RAW 메시지 수신됨! servo{i+1}_raw={value}")

                        # self.get_logger().info(f"SERVO_OUTPUT_RAW 메시지 수신됨! {len(actuator_msg.data)}개 서보 값")
                    elif msg_type == 'ACTUATOR_OUTPUT_STATUS':
                        self.last_actuator_output_status = msg
                        self.get_logger().warn(f"Received ACTUATOR_OUTPUT_STATUS: active={msg.active}, actuator={msg.actuator}")
                        
                        # ACTUATOR_OUTPUT_STATUS -> Float64MultiArray 발행
                        from std_msgs.msg import Float64MultiArray
                        actuator_msg = Float64MultiArray()
                        actuator_msg.data = []
                        
                        # active 필드를 첫번째 항목으로 추가
                        actuator_msg.data.append(float(msg.active))
                        
                        # 32개 채널값을 리스트에 추가
                        for i in range(min(len(msg.actuator), 32)):
                            actuator_msg.data.append(float(msg.actuator[i]))
                        
                        self.pixhawk_actuator_pub.publish(actuator_msg)
                        self.get_logger().info(f"Published actuator values with active={msg.active}")
                    elif msg_type == 'MISSION_ITEM_INT':
                        self.last_mission_item_int = msg

                        # MISSION_ITEM_INT -> GlobalPosition (Target Position) 발행
                        target_msg = GlobalPosition()
                        target_msg.lat = msg.x / 10000000.0  # MAVLink uses int32 * 10^7 for lat/lon
                        target_msg.lon = msg.y / 10000000.0
                        target_msg.alt = msg.z
                        self.target_position_pub.publish(target_msg)
                        
                        # Flight speed 처리 (param2 필드에서 속도 정보 추출)
                        if hasattr(msg, 'param2') and msg.param2 > 0:
                            self.mission_speed_ms = msg.param2
                            self.get_logger().info(f"Mission speed set to: {self.mission_speed_ms} m/s")
                        
                        # 미션 아이템 저장
                        # mission_type 필드가 없거나 MISSION 타입인 경우에만 처리
                        mission_type_ok = True
                        if hasattr(msg, 'mission_type'):
                            mission_type_ok = (msg.mission_type == mavlink.MAV_MISSION_TYPE_MISSION)
                        
                        if hasattr(msg, 'seq') and mission_type_ok:
                            # 저장할 인덱스 확인
                            idx = msg.seq
                            
                            # 필요시 리스트 확장
                            while len(self.mission_items) <= idx:
                                self.mission_items.append(None)
                            
                            # 미션 아이템 저장
                            self.mission_items[idx] = msg
                            self.get_logger().info(f"Received mission item {idx}: lat={msg.x/10000000.0}, lon={msg.y/10000000.0}")
                            
                            # 다음 미션 아이템 요청 (미션 아이템을 모두 받을 때까지)
                            if idx < self.mission_count - 1:
                                self.request_mission_item(idx + 1)
                            else:
                                # 모든 미션 아이템을 받음
                                self.mission_request_in_progress = False
                                self.publish_waypoints()
                    
                    elif msg_type == 'MISSION_COUNT':
                        # 미션 아이템 갯수
                        count = msg.count
                        self.mission_count = count
                        self.get_logger().info(f"Received mission count: {count}")
                        
                        # 초기화
                        self.mission_items = []
                        
                        # 첫번째 미션 아이템 요청
                        if count > 0:
                            self.request_mission_item(0)
                        else:
                            self.get_logger().info("No mission items available")
                            self.mission_request_in_progress = False
                    
                    elif msg_type == 'MISSION_ACK':
                        # 미션 응답
                        if msg.type == mavlink.MAV_MISSION_ACCEPTED:
                            self.get_logger().info("Mission accepted")
                        else:
                            self.get_logger().warning(f"Mission not accepted: {msg.type}")
                        self.mission_request_in_progress = False
                    
                    elif msg_type == 'MISSION_CURRENT':
                        # 현재 실행 중인 미션 항목
                        self.last_mission_current = msg
                        self.get_logger().info(f"Current mission sequence: {msg.seq}")
                        
                        # MISSION_CURRENT -> MissionCurrent 발행
                        from bada_msg.msg import MissionCurrent
                        mission_current_msg = MissionCurrent()
                        mission_current_msg.seq = msg.seq
                        
                        # total 필드를 last_mission_current.total로 설정
                        if hasattr(msg, 'total'):
                            mission_current_msg.total = msg.total
                        else:
                            # 이전 방식으로 fallback
                            if hasattr(self, 'mission_count'):
                                mission_current_msg.total = self.mission_count
                            else:
                                mission_current_msg.total = 0
                        
                        # mission_state 필드를 last_mission_current.mission_state로 설정
                        if hasattr(msg, 'mission_state'):
                            mission_current_msg.mission_state = msg.mission_state
                        
                        self.mission_current_pub.publish(mission_current_msg)
                        self.get_logger().info(f"Published mission_current: seq={mission_current_msg.seq}, total={mission_current_msg.total}, mission_state={mission_current_msg.mission_state if hasattr(msg, 'mission_state') else 'N/A'}")
                        
                    elif msg_type == 'MISSION_ITEM_REACHED':
                        # 도달한 미션 항목
                        self.last_mission_item_reached = msg
                        self.get_logger().info(f"Reached mission sequence: {msg.seq}")
                        
                        # MISSION_ITEM_REACHED -> MissionItemReached 발행
                        from bada_msg.msg import MissionItemReached
                        mission_item_reached_msg = MissionItemReached()
                        mission_item_reached_msg.seq = msg.seq
                        self.mission_item_reached_pub.publish(mission_item_reached_msg)
                        self.get_logger().debug(f"Published mission_item_reached: seq={mission_item_reached_msg.seq}")
                        
                    elif msg_type == 'GPS_RAW_INT':
                        # GPS 정보
                        self.last_gps_raw_int = msg
                        self.get_logger().debug(f"Received GPS_RAW_INT: lat={msg.lat/1e7}, lon={msg.lon/1e7}")
                    
                    elif msg_type == 'RC_CHANNELS':
                        # RC 채널 정보
                        self.get_logger().debug("RC_CHANNELS 메시지 수신됨!")
                        
                        # RC_CHANNELS -> RcChannel 발행
                        rc_msg = RcChannel()
                        
                        # RC_CHANNELS 메시지의 채널 필드들을 배열로 변환
                        channel_fields = [
                            'chan1_raw', 'chan2_raw', 'chan3_raw', 'chan4_raw',
                            'chan5_raw', 'chan6_raw', 'chan7_raw', 'chan8_raw',
                            'chan9_raw', 'chan10_raw', 'chan11_raw', 'chan12_raw',
                            'chan13_raw', 'chan14_raw', 'chan15_raw', 'chan16_raw'
                        ]
                        
                        channels = []
                        for field_name in channel_fields:
                            if hasattr(msg, field_name):
                                channels.append(getattr(msg, field_name))
                            else:
                                channels.append(0)  # 기본값
                        
                        rc_msg.channels = channels
                        self.rc_channels_pub.publish(rc_msg)
                        self.get_logger().debug(f"Published RC channels: {len(channels)} channels")
                        
                else:
                    # 메시지가 없을 때 CPU 과부하 방지를 위한 짧은 대기
                    time.sleep(0.001)  # 1ms 대기
                    
            except Exception as e:
                self.get_logger().error(f"Error receiving MAVLink message: {e}")
                time.sleep(1.0)
    

    def request_mission_count(self):
        """
        PX4로 미션 아이템 갯수 요청
        """
        if self.mavlink_connection is None:
            self.get_logger().warning("MAVLink connection not available, cannot request mission count")
            return False
        
        self.get_logger().info("Requesting mission count from PX4")
        self.mavlink_connection.mav.mission_request_list_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavlink.MAV_MISSION_TYPE_MISSION
        )
        self.mission_request_in_progress = True
        self.mission_last_request_time = time.time()
        return True
    
    def request_mission_item(self, seq):
        """
        특정 미션 아이템 요청
        """
        if self.mavlink_connection is None:
            self.get_logger().warning("MAVLink connection not available, cannot request mission item")
            return False
        
        self.get_logger().info(f"Requesting mission item {seq} from PX4")
        self.mavlink_connection.mav.mission_request_int_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            seq,
            mavlink.MAV_MISSION_TYPE_MISSION
        )
        return True
    
    def mission_request_thread(self):
        """
        10초마다 미션 아이템을 요청하는 스레드
        """
        self.get_logger().info("Starting mission item request thread")
        
        while rclpy.ok():
            try:
                # 10초마다 미션 아이템 요청
                self.mission_speed_ms = 0.0  # 기본 속도 값으로 초기화
                self.request_mission_count()
                # 10초 대기
                time.sleep(10.0)
                
            except Exception as e:
                self.get_logger().error(f"Error in mission request thread: {e}")
                time.sleep(1.0)
    
    def publish_waypoints(self):
        """
        수신한 미션 아이템을 ROS 2 Waypoint 메시지로 발행
        수신한 미션 아이템이 이전에 발행한 미션 아이템과 다를 경우에만 발행
        """
        # 유효한 미션 아이템이 없으면 발행하지 않음
        if not self.mission_items or len(self.mission_items) == 0:
            self.get_logger().warning("No mission items to publish")
            return
            
        # Waypoint 메시지 생성 및 유효한 미션 아이템 추출
        current_waypoints = []
        
        # 모든 미션 아이템을 순회하며 위도/경도 추가
        for item in self.mission_items:
            if item is not None:
                lat = item.x / 10000000.0  # MAVLink uses int32 * 10^7 for lat/lon
                lon = item.y / 10000000.0
                current_waypoints.append((lat, lon))
        
        # 유효한 웨이포인트가 없으면 발행하지 않음
        if len(current_waypoints) == 0:
            self.get_logger().warning("No valid waypoints to publish")
            return
        
        # 이전에 발행한 미션 아이템과 비교 -> 무조건 보내고 controller에서 처리하도록 변경
        # if self._compare_waypoints(current_waypoints, self.last_published_mission_items):
        #     self.get_logger().info("Mission items unchanged, skipping publication")
        #     return
        
        # 새로운 웨이포인트 발행
        waypoint_msg = Waypoint()
        
        # 첫 번째 웨이포인트로 현재 위치 추가
            # 현재 위치 정보가 없으면 기존 방식 사용
        waypoint_msg.x_lat = [wp[0] for wp in current_waypoints]
        waypoint_msg.y_long = [wp[1] for wp in current_waypoints]
        
        total_waypoints = len(waypoint_msg.x_lat)
        waypoint_msg.num_waypoints = total_waypoints
        waypoint_msg.speed_ms = self.mission_speed_ms  # 미션 속도 설정
        self.get_logger().info(f"Publishing {total_waypoints} waypoints (1 current + {len(current_waypoints)} mission points) with speed {self.mission_speed_ms} m/s")
        self.waypoints_pub.publish(waypoint_msg)
        
        # 발행한 웨이포인트 저장
        self.last_published_mission_items = current_waypoints.copy()
    
    def get_flight_mode(self, base_mode, custom_mode):
        """
        PX4 비행 모드를 판별하는 함수
        
        Args:
            base_mode: MAVLink HEARTBEAT 메시지의 base_mode 필드
            custom_mode: MAVLink HEARTBEAT 메시지의 custom_mode 필드
            
        Returns:
            tuple: (flight_mode_enum, mode_name) - 비행 모드 열거형 값과 모드 이름
        """
        # Armed 상태 확인
        is_armed = (base_mode & self.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        armed_str = "ARMED" if is_armed else "DISARMED"
        
        # Custom mode가 활성화되지 않은 경우 base_mode로만 판단
        if not (base_mode & self.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED):
            if base_mode & self.MAV_MODE_FLAG_STABILIZE_ENABLED:
                return self.FLIGHT_MODE["STABILIZE"], f"STABILIZE ({armed_str})"
            if base_mode & self.MAV_MODE_FLAG_GUIDED_ENABLED:
                return self.FLIGHT_MODE["GUIDED"], f"GUIDED ({armed_str})"
            if base_mode & self.MAV_MODE_FLAG_AUTO_ENABLED:
                return self.FLIGHT_MODE["AUTO"], f"AUTO ({armed_str})"
            return self.FLIGHT_MODE["UNKNOWN"], f"UNKNOWN ({armed_str})"
        
        # Custom mode 파싱 (PX4 방식)
        main_mode = (custom_mode >> 0) & 0xFF  # 하위 8비트
        sub_mode = (custom_mode >> 8) & 0xFF   # 다음 8비트
        
        # PX4 main mode에 따른 비행 모드 설정
        if main_mode == self.PX4_CUSTOM_MAIN_MODE_MANUAL:
            return self.FLIGHT_MODE["UNKNOWN"], f"MANUAL ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_STABILIZED:
            return self.FLIGHT_MODE["STABILIZE"], f"STABILIZED ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_ALTCTL:
            return self.FLIGHT_MODE["UNKNOWN"], f"ALTITUDE CONTROL ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_POSCTL:
            return self.FLIGHT_MODE["POSITION"], f"POSITION ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_AUTO:
            # Auto 모드의 sub-mode로 세부 판단
            if sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
                return self.FLIGHT_MODE["AUTO"], f"AUTO MISSION ({armed_str})"
            elif sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                return self.FLIGHT_MODE["GUIDED"], f"AUTO LOITER ({armed_str})"
            elif sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                return self.FLIGHT_MODE["RTL"], f"AUTO RTL ({armed_str})"
            elif sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                return self.FLIGHT_MODE["LAND"], f"AUTO LAND ({armed_str})"
            elif sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                return self.FLIGHT_MODE["UNKNOWN"], f"AUTO TAKEOFF ({armed_str})"
            elif sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
                return self.FLIGHT_MODE["GUIDED"], f"AUTO FOLLOW TARGET ({armed_str})"
            else:
                return self.FLIGHT_MODE["AUTO"], f"AUTO UNKNOWN SUB {sub_mode} ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            return self.FLIGHT_MODE["GUIDED"], f"OFFBOARD ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_ACRO:
            return self.FLIGHT_MODE["UNKNOWN"], f"ACRO ({armed_str})"
            
        elif main_mode == self.PX4_CUSTOM_MAIN_MODE_RATTITUDE:
            return self.FLIGHT_MODE["UNKNOWN"], f"RATTITUDE ({armed_str})"
            
        else:
            return self.FLIGHT_MODE["UNKNOWN"], f"UNKNOWN MODE {main_mode}/{sub_mode} ({armed_str})"

    def print_flight_status(self, base_mode, custom_mode):
        """
        비행 상태 정보를 출력하는 함수
        
        Args:
            base_mode: MAVLink HEARTBEAT 메시지의 base_mode 필드
            custom_mode: MAVLink HEARTBEAT 메시지의 custom_mode 필드
        """
        # 비행 모드 판별
        flight_mode_enum, mode_name = self.get_flight_mode(base_mode, custom_mode)
        
        # Custom mode 파싱 (PX4 방식)
        main_mode = (custom_mode >> 0) & 0xFF  # 하위 8비트
        sub_mode = (custom_mode >> 8) & 0xFF   # 다음 8비트
        
        # Armed 상태 확인
        is_armed = (base_mode & self.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        
        # 상세 상태 정보 출력
        self.get_logger().info("=== PX4 Flight Status ===")
        self.get_logger().info(f"Base Mode: 0x{base_mode:02X} ({base_mode})")
        self.get_logger().info(f"Custom Mode: 0x{custom_mode:08X} ({custom_mode})")
        self.get_logger().info(f"  - Main Mode: {main_mode}")
        self.get_logger().info(f"  - Sub Mode: {sub_mode}")
        self.get_logger().info(f"Flight Mode: {mode_name}")
        self.get_logger().info(f"Armed: {'YES' if is_armed else 'NO'}")
        
        # 특정 모드 체크
        self.get_logger().info("=== Mode Checks ===")
        self.get_logger().info(f"Stabilize Mode: {'YES' if flight_mode_enum == self.FLIGHT_MODE['STABILIZE'] else 'NO'}")
        self.get_logger().info(f"Position Mode: {'YES' if flight_mode_enum == self.FLIGHT_MODE['POSITION'] else 'NO'}")
        self.get_logger().info(f"Mission Mode: {'YES' if (flight_mode_enum == self.FLIGHT_MODE['AUTO'] and sub_mode == self.PX4_CUSTOM_SUB_MODE_AUTO_MISSION) else 'NO'}")
        self.get_logger().info(f"Go to Location: {'YES' if flight_mode_enum == self.FLIGHT_MODE['GUIDED'] else 'NO'}")
    
    def _compare_waypoints(self, waypoints1, waypoints2):
        """
        두 웨이포인트 리스트가 동일한지 비교
        
        Args:
            waypoints1: 첫번째 웨이포인트 리스트 [(lat1, lon1), (lat2, lon2), ...]
            waypoints2: 두번째 웨이포인트 리스트 [(lat1, lon1), (lat2, lon2), ...]
            
        Returns:
            bool: 두 리스트가 동일하면 True, 다르면 False
        """
        # 길이가 다르면 다른 웨이포인트
        if len(waypoints1) != len(waypoints2):
            return False
            
        # 각 웨이포인트 비교 (위도/경도가 같은지)
        for wp1, wp2 in zip(waypoints1, waypoints2):
            # 부동소수점 비교를 위해 작은 오차(epsilon) 허용
            epsilon = 1e-10
            if abs(wp1[0] - wp2[0]) > epsilon or abs(wp1[1] - wp2[1]) > epsilon:
                return False
                
        # 모든 웨이포인트가 동일함
        return True
    
    def publish_estimated_state(self):
        """
        Create and publish a Float64MultiArray with 12 ENU-based items to /ekf/estimated_state.
        """
        try:
            from std_msgs.msg import Float64MultiArray
            import math
            arr = [0.0] * 12

            # 0, 1: GLOBAL_POSITION_INT -> UTM easting, northing (m)
            if self.last_global_position_int:
                lat = self.last_global_position_int.lat / 1e7
                lon = self.last_global_position_int.lon / 1e7
                # easting, northing, _ = self._latlon_to_utm(lat, lon)
                # arr[0] = easting
                # arr[1] = northing

                easting, northing, _, _ = utm.from_latlon(lat, lon)
                arr[0] = easting
                arr[1] = northing
                
                # 3: vx (NED X) -> ENU X (m/s)
                arr[3] = self.last_global_position_int.vx / 100.0 if hasattr(self.last_global_position_int, 'vx') else 0.0
                # 4: vy (NED Y) -> ENU Y (m/s, sway)
                arr[4] = self.last_global_position_int.vy / 100.0 if hasattr(self.last_global_position_int, 'vy') else 0.0
                # 9: sqrt(vx^2 + vy^2) (m/s)
                vx = arr[3]
                vy = arr[4]
                arr[9] = math.sqrt(vx**2 + vy**2)

            # 2, 5, 8, 11: ATTITUDE -> yaw, yaw_rate (ENU 기준)
            if self.last_attitude:
                # PX4: yaw is NED, ENU = pi/2 - yaw
                yaw_ned = self.last_attitude.yaw # mavlink YAW는 단위 : rad, Yaw angle (-pi..+pi)
                yaw_enu = yaw_ned # yaw_enu = math.pi/2 - yaw_ned
                arr[2] = yaw_enu
                arr[8] = yaw_enu
                arr[11] = yaw_enu
                # yaw rate: ENU 기준 (NED 기준은 z, ENU는 -z)
                yaw_rate_ned = self.last_attitude.yawspeed if hasattr(self.last_attitude, 'yawspeed') else 0.0
                arr[5] = -yaw_rate_ned

            # 6, 7, 10: GPS_RAW_INT -> UTM easting, northing, vy sway (m/s)
            if hasattr(self, 'last_gps_raw_int') and self.last_gps_raw_int:
                lat_gps = self.last_gps_raw_int.lat / 1e7
                lon_gps = self.last_gps_raw_int.lon / 1e7
                easting_gps, northing_gps, _ = self._latlon_to_utm(lat_gps, lon_gps)
                arr[6] = easting_gps
                arr[7] = northing_gps
                arr[10] = self.last_gps_raw_int.vel / 100.0 if hasattr(self.last_gps_raw_int, 'vel') else 0.0

            # Create and publish Float64MultiArray
            msg = Float64MultiArray()
            msg.data = arr
            self.estimated_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_estimated_state: {e}")

    def _latlon_to_utm(self, lat, lon):
        """
        Convert latitude and longitude to UTM coordinates (easting, northing, zone).
        Simple implementation as fallback if utm package is not available.
        """
        try:
            # Try to use utm package if available
            import utm
            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
            return easting, northing, f"{zone_number}{zone_letter}"
        except ImportError:
            # Fallback implementation using approximation
            # This is a simplified calculation and not as accurate as utm package
            self.get_logger().warn("utm package not available, using approximation. Install with 'pip install utm'")
            
            # Constants for WGS84
            a = 6378137.0  # equatorial radius in meters
            f = 1.0 / 298.257223563  # flattening
            
            # Convert lat/lon to radians
            import math
            lat_rad = math.radians(lat)
            lon_rad = math.radians(lon)
            
            # Simple approximation - not accurate for large distances
            # For an area of a few km, this approximation is reasonable
            # 1 degree of latitude is approximately 111.32 km
            # 1 degree of longitude varies with latitude
            meters_per_degree_lat = 111320.0
            meters_per_degree_lon = 111320.0 * math.cos(lat_rad)
            
            # Choose arbitrary reference point (0,0 -> somewhere in the Gulf of Guinea)
            # For real applications, use a local reference point
            easting = lon * meters_per_degree_lon
            northing = lat * meters_per_degree_lat
            
            # Fake UTM zone
            return easting, northing, "N/A"
            
    def destroy_node(self):
        # 노드가 종료될 때 자원 정리
        if hasattr(self, 'mavlink_connection') and self.mavlink_connection is not None:
            self.mavlink_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = BadaMavlinkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
