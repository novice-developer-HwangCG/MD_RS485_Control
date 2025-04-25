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

        # 설정할 rpm
        self.test_rpm = 430

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
        wheel_diameter = 0.13  # meters (130mm)
        wheel_circumference = 3.1416 * wheel_diameter  # ≈ 0.408 m
        max_rpm = 430

        # 1. m/s 계산
        mps = (self.test_rpm * wheel_circumference) / 60    # (rpm / 350.0) * 2.03

        # 2. pwm 계산 [비례]
        pwm = int((self.test_rpm / max_rpm) * 255)

        rospy.loginfo(f"[Test RPM: {self.test_rpm}] -> m/s: {mps:.3f}, PWM: {pwm}")

        # 실제 모터로 rpm 명령 전송
        rpm_val = int(self.test_rpm)
        left = -rpm_val
        right = rpm_val
        left_rpm_bytes = list(struct.pack("<h", left))
        right_rpm_bytes = list(struct.pack("<h", right))
        self.send_rs485_command(0xCF, [0x01] + left_rpm_bytes + [0x01] + right_rpm_bytes + [0x00])

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
