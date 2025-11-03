#!/usr/bin/env python3

import threading
from time import sleep
import rclpy
from rclpy.node import Node
import socket
from comm.plc import PLCPacket, PlcToPx4Packet

from aura_msg.msg import ActuatorOutputs
from aura_msg.msg import PlcStatus
from std_msgs.msg import Float64MultiArray
from bada_msg.msg import Mode, RcChannel

class ActuatorSubscriber(Node):

    def __init__(self):
        super().__init__('actuator_subscriber')
        self.px4_listen_port = 2006
        self.plc_ip = '192.168.2.88' # self.plc_ip = '127.0.0.1'
        self.plc_port = 2005
        self.px4_ip = '192.168.2.5' #'127.0.0.1' # self.px4_ip = '192.168.2.200'
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((self.px4_ip, self.px4_listen_port))
        self.plcPacket = PLCPacket()
        
        # 비행 모드와 armed 상태를 저장할 변수 초기화
        self.mode = 0  # FLIGHT_MODE_UNKNOWN
        self.is_armed = 0  # DISARMED
        
        # RC 채널 값 저장 변수 초기화
        self.engine_on = 0     # 6번째 RC 채널 (인덱스 5)
        self.engine_start = 0  # 7번째 RC 채널 (인덱스 6)
        self.last_rc_time = 0.0  # 마지막 RC 메시지 수신 시간
         
        qos2 = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/actuator_outputs',
            self.listener_callback,
            # 10,
            qos_profile=qos2
            )
            
        # 추가: /bada/px4_actuator 토픽 구독
        self.px4_actuator_subscription = self.create_subscription(
            Float64MultiArray,
            '/bada/px4_actuator',
            self.px4_actuator_callback,
            qos_profile=qos2
        )
        
        # RC 채널 토픽 구독
        self.rc_channels_subscription = self.create_subscription(
            RcChannel,
            '/bada/rc_channels',
            self.rc_channels_callback,
            qos_profile=qos2
        )
        
        # 추가: /bada/ros_actuator 토픽 구독
        self.ros_actuator_subscription = self.create_subscription(
            Float64MultiArray,
            '/bada/ros_actuator',
            self.ros_actuator_callback,
            qos_profile=qos2
        )
        
        # 추가: /bada/mode 토픽 구독
        self.mode_subscription = self.create_subscription(
            Mode,
            '/bada/mode',
            self.mode_callback,
            qos_profile=qos2
        )
        
        # RC 채널 타임아웃 확인 타이머 (0.5초마다 실행)
        self.create_timer(0.5, self.check_rc_timeout)
        
        # thread로 0.5초마다 send_read_request 메소드를 실행
        # self.create_timer(0.5, self.send_read_request)
        # thread로 handle_read_response 메소드를 실행
        # thread = threading.Thread(target=self.handle_read_response, args=(self.sock,))
    def listener_callback(self, msg):
        # 수신하면 메시지를 받아서 plc로 전송 
        # socket 생성해서
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        throttle = self.cal_throttle(int(msg.data[1]))# throttle = int(msg.actuator[3])
        steering = self.cal_steering(int(msg.data[0]))
        # throttle = self.cal_throttle(int(1700))# throttle = int(msg.actuator[3])
        # steering = self.cal_steering(int(1700))# steering = int(msg.actuator[1])
        clutch = self.cal_clutch(throttle)
        # print("a")

        sock.sendto(self.plcPacket.makeWritePacket2(int(throttle), int(steering), int(clutch)), (self.plc_ip, self.plc_port))
        print("sending: ", throttle, steering, clutch)
    
    def px4_actuator_callback(self, msg):
        """
        /bada/px4_actuator 토픽에 대한 콜백
        is_armed가 ARM 상태이고, mode가 STABILIZE, MANUAL, 또는 POSITION(POSCTL) 상태일 때만 동작
        """
        # ARM 상태 및 특정 모드 확인
        allowed_modes = [
            Mode.FLIGHT_MODE_STABILIZE,
            Mode.FLIGHT_MODE_MANUAL,
            Mode.FLIGHT_MODE_POSITION
        ]
        # is_armed가 ARM 상태이고 mode가 허용된 모드 중 하나인지 확인
        if self.is_armed != Mode.ARMED or self.mode not in allowed_modes:
            self.get_logger().info(f"PX4 actuator - 무시됨: is_armed={self.is_armed}, mode={self.mode}")
            return
        self.get_logger().info("****** PX4 actuator 수신하여 - 처리 중 *******")    
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        throttle = self.cal_throttle(int(msg.data[3]))# throttle = int(msg.data[3])
        steering = self.cal_steering(int(msg.data[1]))# steering = int(msg.data[1])
        clutch = self.cal_pwm_for_clutch(self.pwm_for_clutch) #clutch = self.cal_clutch(throttle)

        sock.sendto(self.plcPacket.makeWritePacket4(int(throttle), int(steering), int(clutch), int(self.engine_on), int(self.engine_start)), (self.plc_ip, self.plc_port)) #sock.sendto(self.plcPacket.makeWritePacket2(int(throttle), int(steering), int(clutch)), (self.plc_ip, self.plc_port))
        
        self.get_logger().info(f"PX4 actuator - sending: throttle={throttle}, steering={steering}, clutch={clutch}")
    
    def ros_actuator_callback(self, msg):
        """
        /bada/ros_actuator 토픽에 대한 콜백
        is_armed가 ARM 상태이고, mode가 AUTO_MISSION 상태일 때만 동작
        """
        # is_armed가 ARM 상태이고 mode가 AUTO_MISSION인지 확인
        if self.is_armed != Mode.ARMED or self.mode != Mode.FLIGHT_MODE_AUTO_MISSION:
            self.get_logger().debug(f"ROS actuator - 무시됨: is_armed={self.is_armed}, mode={self.mode}")
            return

        self.get_logger().info("ROS actuator 수신하여 - 처리 중 ")    

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 메시지에서 throttle과 steering 값 추출
        if len(msg.data) >= 2:
            if int(msg.data[1]) == 1000 and int(msg.data[0]) == 1000:
                throttle = 0
                steering = 0
                clutch = 0
            else:
                throttle = self.cal_throttle(int(msg.data[1]))
                steering = self.cal_steering(int(msg.data[0]))
                clutch = self.cal_clutch(throttle)
            
            sock.sendto(self.plcPacket.makeWritePacket2(int(throttle), int(steering), int(clutch)), (self.plc_ip, self.plc_port))
            self.get_logger().info(f"ROS actuator - sending: throttle={throttle}, steering={steering}, clutch={clutch}")
        else:
            self.get_logger().warn("ROS actuator message has insufficient data")
            
    def mode_callback(self, msg):
        """
        /bada/mode 토픽에 대한 콜백
        Mode 메시지에서 mode와 is_armed 값을 추출하여 저장
        """
        # Mode 메시지에서 mode와 is_armed 값을 추출
        self.mode = msg.mode
        self.is_armed = msg.is_armed
        self.get_logger().info(f"mode_콜백 : Mode updated: mode={self.mode}, is_armed={self.is_armed}")
        # 로그 출력
        mode_names = {
            Mode.FLIGHT_MODE_UNKNOWN: "UNKNOWN",
            Mode.FLIGHT_MODE_MANUAL: "MANUAL",
            Mode.FLIGHT_MODE_STABILIZE: "STABILIZE",
            Mode.FLIGHT_MODE_POSITION: "POSITION",
            Mode.FLIGHT_MODE_AUTO_LOITER: "AUTO_LOITER",
            Mode.FLIGHT_MODE_AUTO_MISSION: "AUTO_MISSION"
        }
        
        armed_status = "ARMED" if self.is_armed == Mode.ARMED else "DISARMED"
        mode_name = mode_names.get(self.mode, "UNKNOWN")
        
        self.get_logger().info(f"Mode updated: {mode_name} ({self.mode}), Armed status: {armed_status} ({self.is_armed})")

    # 0.5초마다 plc에게 'hello' 메시지를 보내는 send_read_request 메소드를 생성
    def send_read_request(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.px4_ip, self.px4_listen_port))
        sock.sendto(self.plcPacket.makeReadPacket(), (self.plc_ip, self.plc_port))
        print("sending read request")
        
    def handle_read_response(self, sock):
        # plc로부터 응답을 받아서 처리하는 메소드
        # plc로부터 받은 데이터를 출력
        plc_publish = self.create_publisher(PlcStatus, 'plc_status', 10)
        
        px4_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        px4_socket.bind(('', self.px4_listen_port))
        while True:
            data, addr = px4_socket.recvfrom(1024)
            if data[20] == self.plcPacket.RESPONSE_READ:
                if data[19] == self.plcPacket.getCheckSum(data, 0, 19):
                    print('-----PLC RESPONSE_READ Received!!! -------------')
                    plc_packet = PlcToPx4Packet()
                    plc_packet.parseDataBytes(data[32:])
                    plc_packet.printData()
                    # 32번째 index부터 30bytes 데이터를 short int 15개에 담는다.
                    # publish 
                    plc_status = PlcStatus()
                    plc_status.auto_control_status = plc_packet.auto_control_status
                    plc_status.emergency_stop_status = plc_packet.emergency_stop_status
                    plc_status.engine_rpm_status = plc_packet.engine_rpm_status
                    plc_status.clutch_status = plc_packet.clutch_status
                    plc_status.steering_angle_status = plc_packet.steering_angle_status
                    plc_status.trim_angle_status = plc_packet.trim_angle_status
                    plc_status.engine_running_status = plc_packet.engine_running_status
                    plc_status.bow_thruster_power_status = plc_packet.bow_thruster_power_status
                    plc_status.bow_thruster_rev_status = plc_packet.bow_thruster_rev_status
                    plc_publish.publish(plc_status)
            # print(data)
            print(f"Received data: {data} from {addr} length : {len(data)}")

        # data에서 read-request에 대한 응답인지 확인
        # data를 parsing해서 publish
        
        print(data)

    # 1500 일때 0
    # 2000 일때 300
    # 1000 일때 -300
    # 1750 일때 150
    # 1250 일때 -150

    def cal_steering(self, pwm):
        if pwm <= 1550 and pwm >= 1450:
            return 300
        elif pwm > 1550:
            return (pwm - 1550) * 0.66 + 300
        elif pwm < 1450:
            return (pwm - 1000) * 0.66
        else:
            return 300

    # 2000일때 100
    # 1500일때 0
    # 1000 일때 0
    # 1750일때 50

    def cal_throttle(self, pwm):
        if pwm <= 1550 : 
            return 0 
        elif pwm > 1550:
            return (pwm-1550) * 0.26 
        else:
            return 0

    def cal_clutch(self, throttle):
        if throttle < 3:
            return 0
        else:
            return 1

    def cal_pwm_for_clutch(self, pwm_for_clutch):
        """
        PWM 값에 따라 클러치 상태를 결정하는 메서드
        1000 - 1350 사이 값인 경우 : 2
        1350 - 1650 사이 값인 경우 : 0  
        1650 - 2000 사이 값인 경우 : 1
        """
        if 1000 <= pwm_for_clutch < 1350:
            return 2
        elif 1350 <= pwm_for_clutch < 1650:
            return 0
        elif 1650 <= pwm_for_clutch <= 2000:
            return 1
        else:
            # 범위를 벗어난 경우 기본값 반환
            return 0

    def rc_channels_callback(self, msg):
        """
        RC 채널 메시지를 수신하여 engine_on과 engine_start 값 업데이트
        6번째 인덱스 값이 1800보다 크면 engine_on = 1, 작으면 0
        7번째 인덱스 값이 1800보다 크면 engine_start = 1, 작으면 0
        """
        import time
        
        try:
            if len(msg.channels) >= 8:  # 최소 8개 채널 필요 (0-7 인덱스)
                # 마지막 RC 메시지 수신 시간 업데이트
                self.last_rc_time = time.time()
                
                # 7번째 채널 (인덱스 6) -> engine_on (1800 기준으로 0/1 결정)
                ch7_value = msg.channels[6]
                self.engine_on = 1 if ch7_value > 1800 else 0
                
                # 8번째 채널 (인덱스 7) -> engine_start (1800 기준으로 0/1 결정)
                ch8_value = msg.channels[7]
                self.engine_start = 1 if ch8_value > 1800 else 0
                
                ch9_value = msg.channels[8]
                if ch9_value < 1200:
                    clutch = 2
                elif ch9_value < 1750:
                    clutch = 0
                else:
                    clutch = 1

                # rc 채널 값 로그 출력
                for i, value in enumerate(msg.channels):
                    self.get_logger().info(f"RC Channel {i+1}: {value}")
                
                self.pwm_for_clutch = msg.channels[1]

                self.get_logger().info(f"RC Channels - CH7: {ch7_value} -> Engine ON: {self.engine_on}, CH8: {ch8_value} -> Engine Start: {self.engine_start}")
            else:
                self.get_logger().warning(f"RC channels array too short: {len(msg.channels)} (need at least 8)")
                
        except Exception as e:
            self.get_logger().error(f"Error in RC channels callback: {e}")
    
    def check_rc_timeout(self):
        """
        RC 채널 타임아웃 확인 - 2초 이상 메시지를 받지 못하면 엔진 제어 값을 0으로 초기화
        """
        import time
        
        current_time = time.time()
        if self.last_rc_time > 0 and (current_time - self.last_rc_time) > 2.0:
            if self.engine_on != 0 or self.engine_start != 0:
                self.get_logger().warning("RC channels timeout - resetting engine controls to 0")
                self.engine_on = 0
                self.engine_start = 0
                self.pwm_for_clutch = 1500

def main(args=None):
    rclpy.init(args=args)

    actuator_subscriber = ActuatorSubscriber()

    rclpy.spin(actuator_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actuator_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()