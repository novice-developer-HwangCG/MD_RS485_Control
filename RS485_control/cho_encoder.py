#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

class EncoderRate:
    def __init__(self):
        rospy.init_node("encoder_rate")

        self.last_left = None
        self.last_right= None
        self.last_time = rospy.Time.now()

        rospy.Subscriber("/left_encoder", Int32, self.left_callback)
        rospy.Subscriber("/right_encoder", Int32, self.right_callback)

    def left_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()

        if self.last_left is not None and dt > 0:
            delta = msg.data - self.last_left
            rate = delta / dt
            rospy.loginfo(f"L Encoder: {delta} ticks | {rate:.2f} ticks/sec")

        self.last_left = msg.data
        self.last_time = now

    def right_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()

        if self.last_right is not None and dt > 0:
            delta = msg.data - self.last_right
            rate = delta / dt
            rospy.loginfo(f"R Encoder: {delta} ticks | {rate:.2f} ticks/sec")

        self.last_right = msg.data
        self.last_time = now

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        EncoderRate().run()
    except rospy.ROSInterruptException:
        pass
