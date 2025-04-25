!!! This repository is archived and not actively maintained !!!


<---0410 Updated---> 

- 현재 로봇봇에 적용된 코드는 re_motor_control_485.py 코드 나머지 코드는 확인용으로만 볼 것


<---셋팅 방법--->
- 기존 turtlebot 셋팅과 동일 (ros melodic 설치 후 사용)

- 보드레이트 57600

1. cakin_ws/src에 turtlebot3_teleop_key 패키지 설치 (기존에 설치가 되어 있거나 다른 teleop_key 코드가 있다면 넘어가기)
- git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

2. rs485_motor_control 패키지 catkin_ws에 추가 (추후 추가 예정)

2. 빌드
- catkin_make

3. model 설정
-  echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc


<---구 버전---> 테스트 방법
1. master pc
- roscore

2. robot
- rosrun rs485_motor_control rs485_test_node.py
(여기 안돌아가면 catkin_make 하고 source ~/catkin_ws/devel/setup.bash후에 다시 실행)

- roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
(or python3 turtlebot3_teleop_key.py - 해당 파일은 sbc마다 다름 / 신C01 - pid 디렉터리 내부 위치)

3. encoder 값 토픽 확인
- rostopic echo /left_encoder
- rostopic echo /right_encoder


<---0328 Updated---> 신규 테스트 방법
1. master pc
- roscore

2. robot
- sbc 내부에는 rs485_test_node.py 코드 파일 하나 밖에 없음 
- 다른 코드를 테스트하고 싶다면 sbc 바탕화면에 RS485_control 디렉터리 이동
- 해당 디렉터리 내에 테스트 하고 싶은 코드를 rs485_test_node.py에 복붙
[
    RS485_manual_test.py = MDROS의 계산 공식 이용 수동 테스트 (Encoder 값 출력, 단 누적 Encoder 값) ← 주로 사용해야 할 코드
    RS485_MS_to_RPM.py = m/s 값을 설정하여 rpm값 측정 (m/s to RPM, PWM은 비례, 수동 조작 X)
    RS485_RPM_to_PWM = RPM 값을 설정하여 m/s값 측정 (RPM to m/s, PWM은 비례, 수동 조작 X)
    RS485_test_node.py = 외부 사이트 공식 적용 수동 테스트 (Encoder 값 출력, 단 누적 Encoder 값)
]
- rosrun rs485_motor_control rs485_test_node.py
(여기 안돌아가면 catkin_make 하고 source ~/catkin_ws/devel/setup.bash후에 다시 실행)

- Encoder 값을 출력하는 RS485_manual_test.py와 RS485_test_node.py는 실행 시 노란 문구의 경고 메세지가 출력됨
  (Failed to find valid packet header, resyncing...)
- 해당 메세지는 Encoder의 패킷을 찾지 못하는 것으로 turtlebot3_teleop_key를 실행하면 자동으로 사라짐, 즉 문제 없음 

- roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
(or python3 turtlebot3_teleop_key.py - 해당 파일은 sbc마다 다름 / 신C01 - pid 디렉터리 내부 위치)

3. encoder 값 토픽 확인
- rostopic echo /left_encoder
- rostopic echo /right_encoder

※ 번 외
- 아래 ver.x 코드들은 이제 사용 X 참고 용도로만 사용
- cho_encoder.py코드는 엔코더 값을 받아서 초당 엔코더 값을 계산하는 코드
- avr_encoder.py코드는 엔코더 값을 받아서 초당 엔코더 값을 계산 + 종료 키 누를시 초당 엔코더 값을 평균값으로 출력하는 코드

<<<--- !!! 엔코더 중요 --->>>
통신 패킷 요청 값 기존 self.send_rs485_command(0xD2, [0x00])
-> 쓰기용 명령으로 읽기를 시도

통신 패킷 요청 값 변경 self.send_rs485_command(0x04, [0xD2])
-> PID 210의 값 요청

즉 기존 코드는 데이터를 쓰겠다는 의미로 엔코더 데이터를 요청 하지만 0xD2 패킷은 읽기 전용 데이터라 사용 불가
변경된 코드는 데이터 값을 읽겠다는 의미로 엔코더 데이터를 요청 0x04 패킷은 0xD2의 값을 읽고 싶다라는 명령을 전송해서 엔코더 데이터를 받아옴

+ 수신 응답 대기 필요 rospy.sleep(0.01)

● 동일한 사양에서 다른 로봇은 되는데 왜 안되냐
1. 모터 드라이버의 "PID 자동 응답" 기능이 켜져 있었을 가능성
'CMD_MONITOR_BC_ON (0x0B) or PID_MONITOR_BC_STATE = 1' -> self.send_rs485_command(0x10, [0x0B])  # PID_COMMAND, CMD_MONITOR_BC_ON
모터 드라이버가 주기적으로 다음과 같은 데이터를 알아서 브로드캐스트
PID 193: PID_MAIN_DATA
PID 194: PID_IO_MONITOR
PID 196: PID_MONITOR ← 여기 포함된 값에 엔코더 있음

2. 그 로봇에서는 0xD2가 읽기 가능한 PID로 커스터마이징 되어 있었을 가능성
MDROBOT 드라이버는 내부 펌웨어에 따라 특정 PID는 "쓰기+읽기"가 가능하도록 열려 있는 경우



<--- ver 1 모터 하나 테스트--->

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("rs485_motor_controller", anonymous=True)

        # Launch 파일 파라미터 가져오기
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)
        self.wheel_length = rospy.get_param("~wheel_length", 0.41)
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 430)
        self.enable_encoder = rospy.get_param("~enable_encoder", 1)
        self.encoder_ppr = rospy.get_param("~encoder_PPR", 4096)
        self.slow_start = rospy.get_param("~slow_start", 300)
        self.slow_down = rospy.get_param("~slow_down", 300)

        # RS485 포트 설정
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        # ROS 구독자 설정 (/cmd_vel 토픽 수신)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.send_rs485_command(0x05, [0x00])  # 모터 활성화
        self.send_rs485_command(0x11, [0x00])  # RUN/BRAKE 핀 비활성화

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x  # 전진/후진 속도
        angular_z = msg.angular.z  # 회전 속도

        # 속도 변환 (m/s → RPM)
        rpm = (linear_x / (self.wheel_radius * 3.14)) * 60 / self.reduction
        rpm = max(min(rpm, self.maxrpm), -self.maxrpm)  # 속도 제한

        # 방향 반전 적용
        if self.reverse_direction:
            rpm = -rpm

        rpm_bytes = list(struct.pack(">h", int(rpm)))  # 2바이트 데이터

        # (PID 130) 0x82 = left, 0x83 = right
        self.send_rs485_command(0x82, rpm_bytes)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RS485MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


<--- ver 2 좌우 모터 회전 불일치--->

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("rs485_motor_controller", anonymous=True)

        # Launch 파일 파라미터 가져오기
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)
        self.wheel_length = rospy.get_param("~wheel_length", 0.41)
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 430)
        self.enable_encoder = rospy.get_param("~enable_encoder", 1)
        self.encoder_ppr = rospy.get_param("~encoder_PPR", 4096)
        self.slow_start = rospy.get_param("~slow_start", 300)
        self.slow_down = rospy.get_param("~slow_down", 300)

        # RS485 포트 설정
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        # ROS 구독자 설정 (/cmd_vel 토픽 수신)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.send_rs485_command(0x05, [0x00])  # 모터 활성화
        self.send_rs485_command(0x11, [0x00])  # RUN/BRAKE 핀 비활성화

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x  # 전진/후진 속도
        angular_z = msg.angular.z  # 회전 속도

        # 속도 변환 (m/s → RPM)
        rpm = (linear_x / (self.wheel_radius * 3.14)) * 60 / self.reduction
        rpm = max(min(rpm, self.maxrpm), -self.maxrpm)  # 속도 제한

        # 방향 반전 적용
        if self.reverse_direction:
            rpm = -rpm

        rpm_bytes = list(struct.pack(">h", int(rpm)))  # 2바이트 데이터

        # (PID 130) 0x82 = left, 0x83 = right
        self.send_rs485_command(0x82, rpm_bytes)
        self.send_rs485_command(0x83, rpm_bytes)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RS485MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


<--- ver 3 좌우 모터 회전 불일치--->

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)
        self.wheel_length = rospy.get_param("~wheel_length", 0.41)
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 430)

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.send_rs485_command(0x05, [0x00])  # 모터 활성화
        self.send_rs485_command(0x11, [0x00])  # RUN/BRAKE 핀 비활성화

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x  # 전진/후진 속도
        angular_z = msg.angular.z  # 회전 속도

        # 속도 변환 (m/s → RPM)
        base_rpm = (linear_x / (self.wheel_radius * 3.14)) * 60 / self.reduction
        base_rpm = max(min(base_rpm, self.maxrpm), -self.maxrpm)  # 속도 제한

        # 좌우 바퀴 속도 조정
        left_rpm = -base_rpm  # 좌측 바퀴 RPM 반전
        right_rpm = base_rpm  # 우측 바퀴 RPM 유지

        # 바이트 변환
        left_bytes = list(struct.pack(">h", int(left_rpm)))
        right_bytes = list(struct.pack(">h", int(right_rpm)))

        rospy.loginfo(f"Sending Left Motor: RPM = {int(left_rpm)}, Packet = {left_bytes}")
        rospy.loginfo(f"Sending Right Motor: RPM = {int(right_rpm)}, Packet = {right_bytes}")

        # 좌측 모터 (Motor 1)
        self.send_rs485_command(0x82, left_bytes)
        # 우측 모터 (Motor 2)
        self.send_rs485_command(0x83, right_bytes)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RS485MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


<--- ver 4 좌우 모터 같은 방향으로 회전 (rpm 값 수정 필요) --->

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)
        self.wheel_length = rospy.get_param("~wheel_length", 0.41)
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 300)
        self.minrpm = 20  # 최소 RPM 설정

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 모터 타입 설정 (MDH100 사용 설정)
        self.send_rs485_command(0x4F, [0x03])

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        base_rpm = (linear_x / (self.wheel_radius * 3.14)) * 60 / self.reduction
        base_rpm = max(min(base_rpm, self.maxrpm), -self.maxrpm)

        # 좌우 바퀴 방향 수정 (좌측 반시계, 우측 시계)
        left_rpm = -base_rpm  # 좌측 바퀴는 반시계 방향 (그대로 유지)
        right_rpm = base_rpm  # 우측 바퀴는 시계 방향 (부호 반전)

        # 최소 속도 보정 (너무 작은 값 무시 방지)
        if 0 < abs(left_rpm) < self.minrpm:
            left_rpm = self.minrpm if left_rpm > 0 else -self.minrpm
        if 0 < abs(right_rpm) < self.minrpm:
            right_rpm = self.minrpm if right_rpm > 0 else -self.minrpm

        # 바이트 변환 (부호 유지)
        left_speed = list(struct.pack(">h", int(left_rpm)))
        right_speed = list(struct.pack(">h", int(right_rpm)))

        # 방향 플래그 적용
        left_enable = 0x01 if left_rpm > 0 else 0x02  # 0x02 → 반시계
        right_enable = 0x01 if right_rpm > 0 else 0x02  # 0x02 → 반시계

        rospy.loginfo(f"Sending Left Motor: RPM = {int(left_rpm)}, Packet = {left_speed}, Enable = {left_enable}")
        rospy.loginfo(f"Sending Right Motor: RPM = {int(right_rpm)}, Packet = {right_speed}, Enable = {right_enable}")

        # PID 207 사용하여 두 모터 동시에 제어
        self.send_rs485_command(0xCF, [left_enable] + left_speed + [right_enable] + right_speed + [0x00])

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RS485MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


<------------ ver 5 전진 / 후진만 가능 좌우 회전 추가 필요 ------------>

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)
        self.wheel_length = rospy.get_param("~wheel_length", 0.40)
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 430)  # 최대 RPM (MDH100)

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 모터 타입 설정 (MDH100 사용 설정)
        self.send_rs485_command(0x4F, [0x03])

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x * 1.5   # 1.5배수 적용 최대 322rpm 출력 (이론 430, 실제 약 350, 계산 및 테스트 322)
        angular_z = msg.angular.z

        rospy.loginfo(f"Sub cmd_vel = {linear_x}")

        # **RPM 계산 공식 수정 (2π 적용)**
        base_rpm = (linear_x / (self.wheel_radius * 6.283185)) * 60  # 2π 적용

        # **RPM 상한 보정 (0.26 m/s에서 430 RPM)**
        scale_factor = 0.563  # 보정 계수
        base_rpm *= scale_factor  # 보정 적용

        # **최대 RPM 제한 후 0.1 RPM 변환 적용**
        base_rpm = max(min(base_rpm, self.maxrpm), -self.maxrpm)  # 최대 RPM 제한
        left_rpm = -base_rpm
        right_rpm = base_rpm

        left_rpm *= 10  # 0.1 RPM 단위 적용
        right_rpm *= 10

        # **바이트 변환 (리틀 엔디안)**
        left_speed = list(struct.pack("<h", int(left_rpm)))
        right_speed = list(struct.pack("<h", int(right_rpm)))

        # 방향 플래그 적용
        left_enable = 0x01 if left_rpm > 0 else 0x02  # 0x02 → 반시계
        right_enable = 0x01 if right_rpm > 0 else 0x02  # 0x02 → 반시계

        rospy.loginfo(f"Sending Left Motor: RPM = {int(left_rpm)}, Packet = {left_speed}, Enable = {left_enable}")
        rospy.loginfo(f"Sending Right Motor: RPM = {int(right_rpm)}, Packet = {right_speed}, Enable = {right_enable}")

        # PID 207 사용하여 두 모터 동시에 제어
        self.send_rs485_command(0xCF, [left_enable] + left_speed + [right_enable] + right_speed + [0x00])

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RS485MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


<------------ ver 6 전후좌우 반전 수정 필요 ------------>

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)  # 바퀴 반지름
        self.wheel_length = rospy.get_param("~wheel_length", 0.40)   # 바퀴 간 거리
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 430)  # 최대 RPM (MDH100)

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 모터 타입 설정 (MDH100 사용 설정)
        self.send_rs485_command(0x4F, [0x03])

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x * 1.5  # 1.5배수 적용 (최대 속도 증가)
        angular_z = msg.angular.z

        rospy.loginfo(f"Sub cmd_vel = linear_x: {linear_x}, angular_z: {angular_z}")

        # ** 직진 속도 → RPM 변환 (2π 적용)**
        base_rpm = (linear_x / (self.wheel_radius * 6.283185)) * 60

        # ** RPM 상한 보정 (0.26 m/s에서 430 RPM)**
        scale_factor = 0.563  # 보정 계수
        base_rpm *= scale_factor  # 보정 적용

        # ** `angular_z` 적용: 좌/우 바퀴 속도 차이 만들기**
        angular_rpm = (angular_z * self.wheel_length) / (2 * self.wheel_radius) * 60
        rospy.loginfo(f"Angular RPM adjustment: {angular_rpm}")

        # 좌우 바퀴 속도 계산
        left_rpm = base_rpm - angular_rpm  # 좌회전 시 왼쪽 바퀴 감속
        right_rpm = base_rpm + angular_rpm  # 좌회전 시 오른쪽 바퀴 가속

        # 최대 RPM 제한
        left_rpm = max(min(left_rpm, self.maxrpm), -self.maxrpm)
        right_rpm = max(min(right_rpm, self.maxrpm), -self.maxrpm)

        # 0.1 RPM 단위 적용 (컨트롤러 단위 변환)
        left_rpm *= 10
        right_rpm *= 10

        # 바이트 변환 (리틀 엔디안)
        left_speed = list(struct.pack("<h", int(left_rpm)))
        right_speed = list(struct.pack("<h", int(right_rpm)))

        # 방향 플래그 적용
        left_enable = 0x01 if left_rpm > 0 else 0x02  # 0x02 → 반시계
        right_enable = 0x01 if right_rpm > 0 else 0x02  # 0x02 → 반시계

        rospy.loginfo(f"Sending Left Motor: RPM = {int(left_rpm)}, Packet = {left_speed}, Enable = {left_enable}")
        rospy.loginfo(f"Sending Right Motor: RPM = {int(right_rpm)}, Packet = {right_speed}, Enable = {right_enable}")

        # PID 207 사용하여 두 모터 동시에 제어
        self.send_rs485_command(0xCF, [left_enable] + left_speed + [right_enable] + right_speed + [0x00])

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RS485MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


<------------ ver 7 전 후 좌 우 가능 하지만 좌 우 회전에 대한 움직임 부족 수정 필요 ------------>

#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class RS485MotorController:
    def __init__(self):
        rospy.init_node("rs485_motor_controller", anonymous=True)

        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)  # 바퀴 반지름
        self.wheel_length = rospy.get_param("~wheel_length", 0.40)   # 바퀴 간 거리
        self.reduction = rospy.get_param("~reduction", 4.33)
        self.reverse_direction = rospy.get_param("~reverse_direction", 0)
        self.maxrpm = rospy.get_param("~maxrpm", 430)  # 최대 RPM (MDH100)
        self.max_angular_rpm = 18  # 최대 회전 속도

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 모터 타입 설정 (MDH100 사용 설정)
        self.send_rs485_command(0x4F, [0x03])

        rospy.loginfo(f"RS485 Motor Controller Node Started (Port: {self.port}, Baudrate: {self.baudrate})")

    def calculate_checksum(self, data):
        """ 체크섬 계산 """
        checksum = (~sum(data) & 0xFF) + 1
        return checksum & 0xFF

    def send_rs485_command(self, pid, data):
        """ RS485 패킷 생성 및 전송 """
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        rospy.loginfo(f"Sent: {packet}")

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 속도 명령 전송 """
        linear_x = msg.linear.x * 1.5
        angular_z = msg.angular.z

        rospy.loginfo(f"Sub cmd_vel = linear_x: {linear_x}, angular_z: {angular_z}")

        # **RPM 계산 공식 (2π 적용)**
        base_rpm = (linear_x / (self.wheel_radius * 6.283185)) * 60

        # **RPM 보정**
        scale_factor = 0.563
        base_rpm *= scale_factor

        # **최대 RPM 제한 후 0.1 RPM 변환 적용**
        base_rpm = max(min(base_rpm, self.maxrpm), -self.maxrpm)
        left_rpm = -base_rpm
        right_rpm = base_rpm

        left_rpm *= 10
        right_rpm *= 10

        # **angular_rpm 값 동적 계산**
        if angular_z != 0:
            angular_rpm = (abs(angular_z) / 1.82) * self.max_angular_rpm * 10
        else:
            angular_rpm = 0

        # **좌우 회전 적용**
        if angular_z > 0:
            left_rpm += angular_rpm
            right_rpm += angular_rpm
        elif angular_z < 0:
            left_rpm -= angular_rpm
            right_rpm -= angular_rpm

        # **RPM 값 제한**
        left_rpm = max(min(left_rpm, self.maxrpm * 10), -self.maxrpm * 10)
        right_rpm = max(min(right_rpm, self.maxrpm * 10), -self.maxrpm * 10)

        # **바이트 변환 (리틀 엔디안)**
        left_speed = list(struct.pack("<h", int(left_rpm)))
        right_speed = list(struct.pack("<h", int(right_rpm)))

        rospy.loginfo(f"Sending Left Motor: RPM = {int(left_rpm)}, Packet = {left_speed}")
        rospy.loginfo(f"Sending Right Motor: RPM = {int(right_rpm)}, Packet = {right_speed}")

        # PID 207 사용하여 두 모터 동시에 제어
        self.send_rs485_command(0xCF, [0x01] + left_speed + [0x01] + right_speed + [0x00])

    def stop_motors(self):
        """ Ctrl + C 종료 시 모터 정지 """
        rospy.loginfo("Stopping motors...")
        
        # **RPM 0을 전송하여 모터 정지**
        zero_speed = list(struct.pack("<h", 0))
        self.send_rs485_command(0xCF, [0x01] + zero_speed + [0x01] + zero_speed + [0x00])

        # **또는 모터를 완전히 비활성화 (Disable)**
        # self.send_rs485_command(0xCF, [0x00, 0x00, 0x00, 0x00, 0x00])

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = RS485MotorController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.stop_motors()


<------------ ver 8 전 후 좌 우 가능, 엔코더 값 출력 (topic으로도 확인 가능) ------------>


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


<------------ No longer maintained ------------>