#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

class EncoderRate:
    def __init__(self):
        rospy.init_node("encoder_rate")

        self.last_left = None
        self.last_right = None
        self.last_time = rospy.Time.now()

        self.total_left_delta = 0
        self.total_right_delta = 0
        self.total_time_left = 0.0
        self.total_time_right = 0.0

        rospy.Subscriber("/left_encoder", Int32, self.left_callback)
        rospy.Subscriber("/right_encoder", Int32, self.right_callback)

        rospy.on_shutdown(self.on_shutdown)

    def left_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()

        if self.last_left is not None and dt > 0:
            delta = msg.data - self.last_left
            rate = delta / dt
            rospy.loginfo(f"L Encoder: {delta} ticks | {rate:.2f} ticks/sec")
            self.total_left_delta += delta
            self.total_time_left += dt

        self.last_left = msg.data
        self.last_time = now

    def right_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()

        if self.last_right is not None and dt > 0:
            delta = msg.data - self.last_right
            rate = delta / dt
            rospy.loginfo(f"R Encoder: {delta} ticks | {rate:.2f} ticks/sec")
            self.total_right_delta += delta
            self.total_time_right += dt

        self.last_right = msg.data
        self.last_time = now

    def on_shutdown(self):
        avg_left = self.total_left_delta / self.total_time_left if self.total_time_left > 0 else 0
        avg_right = self.total_right_delta / self.total_time_right if self.total_time_right > 0 else 0

        rospy.loginfo("\n======= Encoder Rate Summary =======")
        rospy.loginfo(f"Average Left : {avg_left:.2f} ticks/sec")
        rospy.loginfo(f"Average Right : {avg_right:.2f} ticks/sec")
        rospy.loginfo("====================================")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        EncoderRate().run()
    except rospy.ROSInterruptException:
        pass
