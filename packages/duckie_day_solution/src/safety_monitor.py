#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from std_msgs.msg import Float32, String
import json

class SafetyMonitor:
    def __init__(self):
        rospy.init_node('safety_monitor', log_level=rospy.INFO)
        self.veh = rospy.get_param('~veh', 'default')

        self.speed = 0.0
        self.steering = 0.0
        self.ticks_left = 0
        self.ticks_right = 0
        self.max_speed = rospy.get_param('~max_speed', 0.5)
        self.max_steering = rospy.get_param('~max_steering', 1.0)
        self.wheel_diff_threshold = rospy.get_param('~wheel_diff_threshold', 1000)

        self.cmd_sub = rospy.Subscriber(
            f'/{self.veh}/lane_controller_node/car_cmd',
            Twist2DStamped, self.cmd_callback, queue_size=1
        )
        self.ticks_sub = rospy.Subscriber(
            f'/{self.veh}/wheel_encoder_node/tick',
            BoolStamped, self.tick_callback, queue_size=10
        )
        self.cmd_pub = rospy.Publisher(
            f'/{self.veh}/safety_car_cmd', Twist2DStamped, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            f'/{self.veh}/safety_status', String, queue_size=1, latch=True
        )
        self.rate = rospy.Rate(20)
        self.ticks_left = 0
        self.ticks_right = 0
        self.last_tick_time = rospy.Time.now()

        rospy.Timer(rospy.Duration(0.05), self.monitor_loop)
        rospy.loginfo(f"[{self.veh}] Safety Monitor started (max_speed={self.max_speed})")

    def cmd_callback(self, msg):
        self.speed = abs(msg.v)
        self.steering = abs(msg.omega)

    def tick_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_tick_time).to_sec()
        self.last_tick_time = now
        if msg.data:
            self.ticks_left += 1
        else:
            self.ticks_right += 1

    def monitor_loop(self, event=None):
        issues = []
        if self.speed > self.max_speed:
            issues.append(f"SPEED_LIMIT_EXCEEDED: {self.speed:.2f} > {self.max_speed}")
        if self.steering > self.max_steering:
            issues.append(f"STEERING_LIMIT_EXCEEDED: {self.steering:.2f}")
        diff = abs(self.ticks_left - self.ticks_right)
        if diff > self.wheel_diff_threshold:
            issues.append(f"WHEEL_MISMATCH: diff={diff}")
        if issues:
            status = json.dumps({"status": "WARNING", "issues": issues})
            self.status_pub.publish(String(data=status))
            rospy.logwarn(f"[{self.veh}] {'; '.join(issues)}")

if __name__ == '__main__':
    try:
        sm = SafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
