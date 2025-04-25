#!/usr/bin/python2


"""
* Motor Control Source for differential control
"""
import time
import serial
import rospy
import struct
import math
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, TransformStamped, PoseWithCovarianceStamped, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler as e2q
from tf.transformations import euler_from_quaternion as q2e
import tf2_ros
import tf

#p : 0.84 -> almost 360 degree

""" PARAMETERS """    
# pid gain   
Kp = 0.0
Ki = 0.0
Kd = 0.0 #0.01

# time interval
#dt = 1

# car parameters
Len_bw_wheel = 0.468 # [m]
Vel_max = 2.03

class MotorController:
    def __init__(self):
        self.current_time = rospy.Time.now()
        
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB2")
        self.baudrate = rospy.get_param("~serial_baudrate", 57600)
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.vel = 0   #default 0.5
        self.angular_vel = 0
        self.encoder_vel = 0
        self.encoder_angular_vel = 0
        
        self.send_rs485_command(0x0A, [0x0A])
        rospy.sleep(0.05)
        self.read_encoder_response()

        self.current_left_encoder = 0
        self.current_right_encoder = 0

        print(self.current_left_encoder, self.current_right_encoder)

        self.left_tick_per_sec = 0
        self.right_tick_per_sec = 0

        self.encoder_last_calc_time = rospy.Time.now()
        self.encoder_last_left = 0
        self.encoder_last_right = 0
    
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.send_motor_mode()
        self.encoder_rate_timer = rospy.Timer(rospy.Duration(1.0), self.encoder_tick_rate_callback)

        rospy.on_shutdown(self.stop_motor)
        
    """ ----------------- PUB&SUB ---------------- """
    def send_motor_mode(self):
        self.send_rs485_command(0x4F, [0x03]) 

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            if isinstance(byte, str):
                checksum+=ord(byte)
            else:
                checksum+=byte
                
        return ((~checksum & 0xFF)+1) & 0xFF

    # def calculate_checksum(self, data):
    #     data_int = [ord(b) if isinstance(b, str) else b for b in data]
    #     return ((~sum(data_int) & 0xFF) + 1) & 0xFF

    def send_rs485_command(self, pid, data):
        packet = [0xB7, 0xB8, 0x01, pid, len(data)] + data
        packet.append(self.calculate_checksum(packet))
        self.ser.write(bytearray(packet))
        # rospy.loginfo(packet)
        
    def stopper_callback(self, msg):
        if msg.data == 1:
            self.stop = True
        elif msg.data == 0:
            self.stop = False
            
    def check_callback(self, msg):
        if msg.data == 0:
            self.stop = True
            
    def cmd_vel_callback(self, msg):
        self.vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def imu_callback(self, msg):
        # None
        
    def bytes_to_int(self, byte_list):
        result = 0
        for i, b in enumerate(byte_list):
            if isinstance(b, str):  # str = byte
                b = ord(b)
            result |= b << (8 * i)
        if result >= (1 << 31): 
            result -= (1 << 32)
        return result

    def encoder_tick_rate_callback(self, event):
        self.left_tick_per_sec = self.current_left_encoder - self.encoder_last_left
        self.right_tick_per_sec = self.current_right_encoder - self.encoder_last_right

        print(self.left_tick_per_sec, self.right_tick_per_sec)
        
        self.encoder_last_left = self.current_left_encoder
        self.encoder_last_right = self.current_right_encoder

    def read_encoder_response(self):       
        self.send_rs485_command(0x04, [0xD2])
        rospy.sleep(0.01)
        response = self.ser.read(64)
        
        start_idx = response.find(b'\xb8\xb7')       
        if start_idx == -1 or (start_idx + 24) > len(response):
            return
        packet = response[start_idx:start_idx + 24]
        
        if ord(packet[3]) != 0xD2:
            rospy.logwarn("Unexpected PID received: {}".format(ord(packet[3])))
            return
        if not self.verify_checksum(packet):
            rospy.loginfo("Raw packet HEX: {}".format(" ".join(["%02X" % ord(b) for b in packet])))
            return

        left_encoder = self.bytes_to_int(packet[10:14]) * -1
        right_encoder = self.bytes_to_int(packet[19:23])
        
        self.current_left_encoder = left_encoder
        self.current_right_encoder = right_encoder
        
    def verify_checksum(self, packet):
        data = packet[:-1]  
        received_crc = ord(packet[-1]) if isinstance(packet[-1], str) else packet[-1]
        calculated_crc = self.calculate_checksum(data)
        return received_crc == calculated_crc
    
    def send_rpm(self):
        if self.stop:
            rpm_left = 0
            rpm_right = 0
        else:
            wheel_sep = 0.468
            max_rpm = 370.0
            # max_speed = 2.03
            mps_to_rpm = 146.9

            # 선속도에 대한 rpm 변환
            rpm_linear = self.vel * mps_to_rpm
            
            # 과도한 회전 억제를 위한 클리핑 추가 가능
            self.angular_vel = max(min(self.angular_vel, 3.0), -3.0)

            # 각속도에 대한 바퀴 간 속도 차이 → rpm 변환
            rpm_diff = self.angular_vel * wheel_sep * mps_to_rpm

            """
            각속도에 대한 각운동학 공식 내용 (실측 기준)
            
            rpm_diff = ω * L * mps_to_rpm -> 사용중인 공식
            ω: 각속도 (rad/s)
            L: 0.468m: 바퀴 간 거리

            rpm_diff = 370이 되도록 하는 ω (angular.z) 값
            370 = ω * 0.468 * 146.9

            양 변 나누기

            ω = 370 / 0.468 * 146.9 ≈ 370 / 68.738 ≈ 5.38rad/s

            좌우 바퀴가 최대 rpm 차이를 가지려면 각속도는 약 ±5.38 rad/s이어야 함

            teleop 키보드 설정에서 WAFFLE_MAX_ANG_VEL = 5.0 제한 이는 이미 5.0 rad/s로 제한 중이므로 안전범위 내

            <!> 제자리 회전에서 5.0rad/s를 주었는데 rpm 측정 시 두 바퀴가 약 171.85가 측정 되는 이유
            - 이는 충분히 5.0rad/s를 실현하고 있는 것
            - 각속도(angular.z) → rpm을 계산할 때 기준은 "바퀴 간 거리"를 이용한 각운동학 공식
            ω = V(right) - V(left) / L
            
            - 양쪽 바퀴의 선속도 차이만 알면 ω 값이 나옴 그리고 이 선속도는 다시 rpm으로 환산
            rpm_diff = ω * L * mps_to_rpm

            - 'angular vel 값을 최대로 주었으니 각 바퀴 모두가 370rpm으로 돈다' = X
            - 이는 전진, 후진에 대한 개념

            - angular vel 값이 5.0일 때 제자리 회전에서 좌측 바퀴는 정방향, 우측 바퀴는 역방향 그리고 서로 반대 방향으로 똑같이 돌면 그게 최대 회전
            - 즉 좌측 rpm = +171.85, 우측 rpm = -171.85
            - 이 둘의 rpm 차이 = 343.7, 이 값이 바로 5.0 rad/s 회전 속도에 해당하는 결과

            <!> 비교 예시
            직진 2.03m/s, 좌 = -370, 우 = +370, 총합 = 740 rpm -> 양 바퀴 모두 빠르게 전진
            제자리 회전 5.0 rad/s, 좌 = +171.85, 우 = -171.85, 총합 = 343.7 rpm 차 -> 좌우 반대 방향으로 회전

            결론 -> 총 회전량이 중요한 게 아니라, 바퀴 간 차이로 각속도를 만들어낸다는 개념이 핵심
            제자리 회전 시엔 양쪽 바퀴가 정반대 방향으로 돌기 때문에 rpm은 낮아도 충분히 빠르게 회전 가능

            자율은 모르겠지만 수동 조작 시 회전 시에 rpm 값을 더 높이고 싶을 경우 teleop 키보드 설정에서 WAFFLE_MAX_ANG_VEL 값을 10.0으로 수정 단, 너무 빠르게 회전 할 시 불안정성 높음

            """

            rpm_left = -(rpm_linear - rpm_diff / 2.0)
            rpm_right = rpm_linear + rpm_diff / 2.0

            # v_left = self.vel - (self.angular_vel * wheel_sep / 2)
            # v_right = self.vel + (self.angular_vel * wheel_sep / 2)
            # rpm_left = -v_left * mps_to_rpm
            # rpm_right = v_right * mps_to_rpm
            
            # rpm_left = -(v_left / max_speed) * max_rpm
            # rpm_right = (v_right / max_speed) * max_rpm
            rpm_left = max(min(rpm_left, max_rpm), -max_rpm)
            rpm_right = max(min(rpm_right, max_rpm), -max_rpm)
            
        left_speed = list(struct.pack("<h", int(rpm_left)))
        right_speed = list(struct.pack("<h", int(rpm_right)))
        # print(left_speed, right_speed)
        self.send_rs485_command(0xCF, [0x01] + left_speed + [0x01] + right_speed + [0x00])
        
    def calculate_vel(self):
        #left_vel = ((self.left_tick_per_sec / 4096.0) * (2 * math.pi)) * 0.065 * 1 
        #right_vel = ((self.right_tick_per_sec / 4096.0) * (2 * math.pi)) * 0.065
        left_vel = self.left_tick_per_sec * (2 * math.pi * 0.065 / 60.0)
        right_vel = self.right_tick_per_sec * (2 * math.pi * 0.065 / 60.0)
        
        self.encoder_vel = (left_vel + right_vel) / 2.0
        self.encoder_angular_vel = (right_vel - left_vel) / Len_bw_wheel
        #self.encoder_angular_vel = (0.065 / 0.468) * ((right_vel - left_vel) / 4096.0) * (2 * math.pi)
        
    def pub_odom(self):
        # calculate duration

        # calculate deviation

        #use imu 
        
        # use not imu
        
        # calculate current position

        # base_link <-> odom braodcaster

        # produce Odometry

        #ekf package use -> below covariance not use
        
    def stop_motor(self):
        self.zero_rpm=0
        zero_rpm_val = int(self.zero_rpm)
        zero_rpm_bytes = list(struct.pack("<h", zero_rpm_val))
        for _ in range(3):
            if self.ser and self.ser.is_open:
                try:
                    self.send_rs485_command(0xCF, [0x01] + zero_rpm_bytes + [0x01] + zero_rpm_bytes + [0x00])
                    rospy.sleep(0.1)
                except serial.SerialException as e:
                    rospy.logwarn("Serial exception during stop_motor: {}".format(e))
            else:
                rospy.logwarn("Serial port already closed, cannot send stop command.")
                break
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            rospy.logwarn("Exception while closing serial port: {}".format(e))
        
    """ ------------------ MAIN ----------------- """        
    def main(self):
        rospy.loginfo("Start publish encoder data with Odometry...")
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            self.read_encoder_response()
            self.calculate_vel() 
            
            self.pub_odom()
            self.send_rpm()
                        
            rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("motor_control")
    #rospy.init_node("checker")
    mc = MotorController()
    try:
        mc.main()
    except KeyboardInterrupt:
        mc.stop_motor()
        #rospy.on_shutdown()
    except Exception as e:
        mc.stop_motor()
        print("except!!!")
        print(e)
        #rospy.on_shutdown()
