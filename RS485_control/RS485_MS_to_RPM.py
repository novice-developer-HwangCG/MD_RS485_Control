#!/usr/bin/env python3
import rospy
import serial
import struct

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # 모터 설정
        self.send_rs485_command(0x4F, [0x03])
        rospy.loginfo("RS485 Motor Controller Node Started")

        # 설정할 ms
        self.test_speed = 2.03

        # 주기적으로 rpm → m/s, pwm 출력
        self.timer = rospy.Timer(rospy.Duration(1.0), self.display_rpm_info)

        rospy.on_shutdown(self.stop_motor)

    def calculate_checksum(self, data):
        return ((~sum(data) & 0xFF) + 1) & 0xFF

    def send_rs485_command(self, pid, data):
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))

    def display_rpm_info(self, event):
        max_rpm = 370.0  # 실측 기준 최대 rpm
        max_speed = 2.03  # 실측 기준 최대 속도 (m/s)

        # 1. m/s → rpm 변환 (보정 적용)
        rpm = (self.test_speed / max_speed) * max_rpm
        rpm = min(rpm, max_rpm)

        # 2. rpm → pwm 계산 [비례]
        pwm = int((rpm / max_rpm) * 255)

        rospy.loginfo(f"[Input Speed: {self.test_speed:.2f} m/s] -> RPM: {rpm:.2f}, PWM: {pwm}")

        # 실제 모터로 rpm 명령 전송
        rpm_val = int(rpm)
        left = -rpm_val
        right = rpm_val
        left_rpm_bytes = list(struct.pack("<h", left))
        right_rpm_bytes = list(struct.pack("<h", right))
        self.send_rs485_command(0xCF, [0x01] + left_rpm_bytes + [0x01] + right_rpm_bytes + [0x00])

        # wheel_diameter = 0.13  # meters
        # wheel_circumference = 3.1416 * wheel_diameter  # ≈ 0.408 m
        # max_rpm = 430

        # # 1. m/s → rpm 변환
        # rpm = (self.test_speed * 60) / wheel_circumference  # RPM = v * 60 / C
        # rpm = min(rpm, max_rpm)

        # # 2. rpm → pwm 계산
        # pwm = int((rpm / max_rpm) * 255)

        # rospy.loginfo(f"[Input Speed: {self.test_speed:.2f} m/s] -> RPM: {rpm:.2f}, PWM: {pwm}")

        # # 실제 모터로 rpm 명령 전송
        # rpm_val = int(rpm)
        # left = -rpm_val
        # right = rpm_val
        # # rpm_bytes = list(struct.pack("<h", rpm_val))
        # left_rpm_bytes = list(struct.pack("<h", left))
        # right_rpm_bytes = list(struct.pack("<h", right))
        # self.send_rs485_command(0xCF, [0x01] + left_rpm_bytes + [0x01] + right_rpm_bytes + [0x00])

    def run(self):
        rospy.spin()

    def stop_motor(self):
        rospy.loginfo("Shutting down... stopping motor.")
        self.timer.shutdown()

        self.zero_rpm=0
        zero_rpm_val = int(self.zero_rpm)
        zero_rpm_bytes = list(struct.pack("<h", zero_rpm_val))

        for _ in range(3):
            self.send_rs485_command(0xCF, [0x01] + zero_rpm_bytes + [0x01] + zero_rpm_bytes + [0x00])
            rospy.sleep(0.1)

        self.ser.close()

if __name__ == "__main__":
    try:
        RS485MotorController().run()
    except rospy.ROSInterruptException:
        pass
