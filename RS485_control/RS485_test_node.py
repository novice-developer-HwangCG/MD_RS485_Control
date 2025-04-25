#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        # 파라미터
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        # 포트 초기화 / 이전 MDAS 프로그램으로 사용된 쓰레기 데이터 초기화 그래도 실행이 안된다면 reboot
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # 퍼블리셔
        self.left_encoder_pub = rospy.Publisher("/left_encoder", Int32, queue_size=10)
        self.right_encoder_pub = rospy.Publisher("/right_encoder", Int32, queue_size=10)

        # 서브스크라이버
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 주기적 엔코더 읽기
        rospy.Timer(rospy.Duration(0.1), self.request_encoder_data)

        # 모터 타입 설정 (MDH100 기준) - PID 79 (0x4f) 메뉴얼 가이드 43page
        self.send_rs485_command(0x4F, [0x03])

        # 엔코더 값 초기화
        self.send_rs485_command(0x7B, [0x00, 0x00])
        self.send_rs485_command(0xAA, [0x00, 0x00])

        # 이전 값 저장 (필터링용)
        self.last_left_encoder = None
        self.last_right_encoder = None

        rospy.loginfo("RS485 Motor Controller Node Started")

    def calculate_checksum(self, data):
        return ((~sum(data) & 0xFF) + 1) & 0xFF

    def send_rs485_command(self, pid, data):
        # 모터 드라이버 설정 - PID 183 (0xb7) 메뉴얼 가이드 6page, 66page
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x * 1.5
        angular_z = msg.angular.z

        rospy.loginfo(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")

        wheel_radius = 0.065
        max_rpm = 430
        base_rpm = (linear_x / (wheel_radius * 6.283185)) * 60
        base_rpm *= 0.563  # 보정 계수
        base_rpm = max(min(base_rpm, max_rpm), -max_rpm)

        left_rpm = -base_rpm
        right_rpm = base_rpm

        if angular_z != 0:
            angular_rpm = (abs(angular_z) / 1.82) * 18 * 10
        else:
            angular_rpm = 0

        if angular_z > 0:
            left_rpm += angular_rpm / 10
            right_rpm += angular_rpm / 10
        elif angular_z < 0:
            left_rpm -= angular_rpm / 10
            right_rpm -= angular_rpm / 10

        left_rpm = max(min(left_rpm, max_rpm), -max_rpm)
        right_rpm = max(min(right_rpm, max_rpm), -max_rpm)
        left_speed = list(struct.pack("<h", int(left_rpm * 10)))
        right_speed = list(struct.pack("<h", int(right_rpm * 10)))

        rospy.loginfo(f"Left RPM: {left_rpm}, Right RPM: {right_rpm}")

        # 모터 속도 제어 명령 PID 207 (0xcf) - 메뉴얼 가이드 86page
        self.send_rs485_command(0xCF, [0x01] + left_speed + [0x01] + right_speed + [0x00])

    def request_encoder_data(self, event):
        # 모터 데이터 요청 PID 210 (0xd2) - 메뉴얼 가이드 87page [읽기용]]
        self.send_rs485_command(0xD2, [0x00])
        # rospy.loginfo("Requesting encoder data...")
        self.read_encoder_response()

    def verify_checksum(self, packet):
        """ 수신 패킷의 체크섬 검증 """
        data = packet[:-1]  # 체크섬 제외하고 계산
        received_crc = packet[-1]
        calculated_crc = self.calculate_checksum(data)
        return received_crc == calculated_crc

    def read_encoder_response(self):
        response = self.ser.read(64)  # 최대한 많이 읽기
        # print("RAW READ:", response.hex())
        start_idx = response.find(b'\xb8\xb7')

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