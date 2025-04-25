#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

"""
/cmd_vel 값 을 받아서 모터에게 전송 m/s → RPM 값 출력
모터 드라이버에게 엔코더 값을 요청 엔코더 값 출력 (메세지 출력 및 각 좌,우 모터 엔코더 값 pub)
"""

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        # param
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        # 포트 초기화 / 이전 MDAS 프로그램으로 사용된 쓰레기 데이터 초기화 그래도 실행이 안된다면 reboot
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # pub
        self.left_encoder_pub = rospy.Publisher("/left_encoder", Int32, queue_size=10)
        self.right_encoder_pub = rospy.Publisher("/right_encoder", Int32, queue_size=10)

        # sub
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # encoder 읽기
        rospy.Timer(rospy.Duration(0.1), self.request_encoder_data)

        # 모터 타입 설정 (MDH100 기준) - PID 79 (0x4f) 메뉴얼 가이드 43page
        self.send_rs485_command(0x4F, [0x03])

        # 엔코더 값 초기화 -> 잘 안됨 - PID 123 (0x7b), 170 (0xAA) 메뉴얼 가이드 51page, 61page
        self.send_rs485_command(0x7B, [0x00, 0x00])
        self.send_rs485_command(0xAA, [0x00, 0x00])

        # 엔코더 이전 값 저장 (필터링용)
        self.last_left_encoder = None
        self.last_right_encoder = None

        rospy.loginfo("RS485 Motor Controller Node Started")

    """데이터가 전송 중에 손상되었는지 확인하기 위한 메서드(중요*)"""
    def calculate_checksum(self, data):
        return ((~sum(data) & 0xFF) + 1) & 0xFF

    """모터 드라이버에게 신호를 보내는 메서드(중요*)"""
    def send_rs485_command(self, pid, data):
        # 모터 드라이버 설정 - PID 183 (0xb7) 메뉴얼 가이드 6page, 66page
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        # rospy.loginfo(f"Sent: {packet}")

    """/cmd_vel topic 받아서 rpm으로 변환하는 메서드"""
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z # 
        """
        angular 값 참고
        teleop_key에서 최대 1.82로 설정되어 있는데 실제 1.82 값으로 설정할 시 두 모터 rpm 최대 68.00 측정됨 -> 정상
        이를 370으로 강제 조정을 원할 시 아래 1, 2번 방안 중 택

        1. 아래 코드를 사용
        ANGULAR_SCALE = 370.0 / ((1.82 * (0.41 / 2)) / 2.03 * 370.0)

        v_left = linear - (angular * wheel_separation / 2 * ANGULAR_SCALE)
        v_right = linear + (angular * wheel_separation / 2 * ANGULAR_SCALE)

        2. teleop_key에 angular.z 값 1.82를 5.5정도로 고쳐 사용
        """

        # rospy.loginfo(f"Received cmd_vel: linear={linear}, angular={angular}")

        # 로봇 파라미터
        wheel_separation = 0.41  # m
        max_rpm = 370.0        # 실측 최대 rpm
        max_speed = 2.03       # 실측 최대 속도 (m/s)

        # 각 바퀴 선속도 (m/s)
        v_left = linear - (angular * wheel_separation / 2)
        v_right = linear + (angular * wheel_separation / 2)

        # 선속도 → RPM
        rpm_left = -(v_left / max_speed) * max_rpm
        rpm_right = (v_right / max_speed) * max_rpm

        # 제한
        rpm_left = max(min(rpm_left, max_rpm), -max_rpm)
        rpm_right = max(min(rpm_right, max_rpm), -max_rpm)

        # 모터에 보낼 데이터 (x10)
        left_speed = list(struct.pack("<h", int(rpm_left)))
        right_speed = list(struct.pack("<h", int(rpm_right)))

        rospy.loginfo(f"Left RPM: {rpm_left:.2f}, Right RPM: {rpm_right:.2f}")

        # PID 207 (0xCF) : 속도 제어 명령
        self.send_rs485_command(0xCF, [0x01] + left_speed + [0x01] + right_speed + [0x00])

    """모터 드라이버에게 엔코더 요청"""
    def request_encoder_data(self, event):
        # 모터 데이터 요청 PID 210 (0xd2) - 메뉴얼 가이드 87page [읽기용]]
        self.send_rs485_command(0xD2, [0x00])
        # rospy.loginfo("Requesting encoder data...")
        self.read_encoder_response()

    """수신 패킷의 체크섬 검증 응답 받은 패킷 확인 (중요*)"""
    def verify_checksum(self, packet):
        data = packet[:-1]  # 체크섬 제외하고 계산
        received_crc = packet[-1]
        calculated_crc = self.calculate_checksum(data)
        return received_crc == calculated_crc

    """엔코더 값 검사 및 필터링 이후 엔코더 값 출력"""
    def read_encoder_response(self):
        response = self.ser.read(64)  # 최대한 많이 읽기
        # print("RAW READ:", response.hex())
        start_idx = response.find(b'\xb8\xb7')

        # 모터가 동작 중일 때만 상태 응답을 하기 때문에 수동 조작 코드나 자율 주행을 실행하지 않으면 아래 메세지만 출력함
        if start_idx == -1 or (start_idx + 24) > len(response):
            rospy.logwarn("Failed to find valid packet header, resyncing...")
            self.ser.reset_input_buffer()
            return

        packet = response[start_idx:start_idx + 24]
        # rospy.loginfo(f"Valid Packet: {packet.hex()}")

        # PID 검사
        if packet[3] != 0xD2:
            rospy.logwarn(f"Unexpected PID received: {packet[3]}")
            return

        # CRC 검사 추가
        if not self.verify_checksum(packet):
            rospy.logwarn("Checksum error detected, dropping packet")
            self.ser.reset_input_buffer()
            return

        # 해당 값들이 이상하게 나온다하면 packet 값을 좌측 [9:13], 우측 [18:22]로 변경해서 테스트
        left_encoder = int.from_bytes(packet[10:14], byteorder='little', signed=True)
        right_encoder = int.from_bytes(packet[19:23], byteorder='little', signed=True)

        # 값 튐 필터링
        if self.last_left_encoder is not None and abs(left_encoder - self.last_left_encoder) > 50000:
            rospy.logwarn(f"Left Encoder Jump Detected - Ignored (prev: {self.last_left_encoder}, new: {left_encoder})")
            left_encoder = self.last_left_encoder
        if self.last_right_encoder is not None and abs(right_encoder - self.last_right_encoder) > 50000:
            rospy.logwarn(f"Right Encoder Jump Detected - Ignored (prev: {self.last_right_encoder}, new: {right_encoder})")
            right_encoder = self.last_right_encoder

        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder

        rospy.loginfo(f"Left Encoder: {left_encoder}, Right Encoder: {right_encoder}")
        self.left_encoder_pub.publish(left_encoder)
        self.right_encoder_pub.publish(right_encoder)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        RS485MotorController().run()
    except rospy.ROSInterruptException:
        pass