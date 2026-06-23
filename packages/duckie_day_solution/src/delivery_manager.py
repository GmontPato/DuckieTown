#!/usr/bin/env python3
import rospy
import json
import os
import actionlib
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    BoolStamped,
    AprilTagDetectionArray,
)
from std_msgs.msg import String, Empty

class DeliveryManager:
    def __init__(self):
        rospy.init_node('delivery_manager', log_level=rospy.INFO)
        self.veh = rospy.get_param('~veh', 'default')
        rospy.loginfo(f"[{self.veh}] Delivery Manager started")

        deliveries_path = rospy.get_param('~deliveries', '/data/deliveries.json')
        self.deliveries = self._load_deliveries(deliveries_path)
        self.current_delivery_idx = 0
        self.delivery_completed = [False] * len(self.deliveries)

        self.state = 'LANE_FOLLOWING'
        self.stopped_at_tag = False
        self.last_tag_id = None
        self.delivery_duration = rospy.Duration(3.0)
        self.delivery_start_time = None

        self.tag_sub = rospy.Subscriber(
            f'/{self.veh}/apriltag_detector_node/detections',
            AprilTagDetectionArray, self.tag_callback, queue_size=1
        )
        self.cmd_pub = rospy.Publisher(
            f'/{self.veh}/lane_controller_node/car_cmd',
            Twist2DStamped, queue_size=1
        )
        self.cmd_stop = rospy.Publisher(
            f'/{self.veh}/joy_mapper_node/joy_override',
            Twist2DStamped, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            f'/{self.veh}/delivery_state',
            String, queue_size=1, latch=True
        )

        self.rate = rospy.Rate(10)
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def _load_deliveries(self, path):
        default = [
            {"id": "QR001", "station": "P1", "feed": "Alimento Salmón A", "kg": 50.0},
            {"id": "QR002", "station": "P2", "feed": "Alimento Salmón B", "kg": 30.0},
            {"id": "QR003", "station": "P3", "feed": "Alimento Salmón C", "kg": 40.0},
        ]
        try:
            with open(path) as f:
                data = json.load(f)
                deliveries = data.get('deliveries', data)
                rospy.loginfo(f"[{self.veh}] Loaded {len(deliveries)} deliveries")
                return sorted(deliveries, key=lambda item: item.get('priority', 999))
        except Exception as e:
            rospy.logwarn(f"[{self.veh}] Could not load deliveries: {e}. Using defaults.")
            return default

    def tag_callback(self, msg):
        if not msg.detections:
            return
        best_tag = msg.detections[0]
        tag_id = f"QR{best_tag.tag_id:03d}"
        self.last_tag_id = tag_id
        self._check_delivery_station(tag_id)

    def _check_delivery_station(self, tag_id):
        for i, delivery in enumerate(self.deliveries):
            expected = delivery.get('id', '')
            if tag_id == expected and not self.delivery_completed[i]:
                if self.current_delivery_idx == i:
                    self._start_delivery(i)
                    return

    def _start_delivery(self, idx):
        rospy.loginfo(f"[{self.veh}] DELIVERY START: Station {self.deliveries[idx]['station']} - {self.deliveries[idx]['feed']}")
        self.state = 'DELIVERING'
        self.stopped_at_tag = True
        self.delivery_start_time = rospy.Time.now()
        self._publish_cmd(0.0, 0.0)
        msg = String(data=json.dumps({
            "action": "delivery_start",
            "station": self.deliveries[idx]['station'],
            "feed": self.deliveries[idx]['feed'],
            "amount_kg": self.deliveries[idx]['kg'],
            "delivery_id": self.deliveries[idx]['id'],
        }))
        self.state_pub.publish(msg)

    def control_loop(self, event=None):
        if self.state == 'DELIVERING' and self.delivery_start_time:
            elapsed = rospy.Time.now() - self.delivery_start_time
            if elapsed >= self.delivery_duration:
                idx = self.current_delivery_idx
                self.delivery_completed[idx] = True
                rospy.loginfo(f"[{self.veh}] DELIVERY COMPLETE: Station {self.deliveries[idx]['station']}")
                msg = String(data=json.dumps({
                    "action": "delivery_complete",
                    "station": self.deliveries[idx]['station'],
                    "delivery_id": self.deliveries[idx]['id'],
                }))
                self.state_pub.publish(msg)
                self.state = 'LANE_FOLLOWING'
                self.current_delivery_idx += 1
                self.stopped_at_tag = False
                all_done = all(self.delivery_completed)
                if all_done:
                    rospy.loginfo(f"[{self.veh}] ALL DELIVERIES COMPLETE!")
                    msg = String(data=json.dumps({"action": "mission_complete"}))
                    self.state_pub.publish(msg)
                self._publish_cmd(0.3, 0.0)

    def _publish_cmd(self, v, omega):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = v
        cmd.omega = omega
        self.cmd_stop.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        dm = DeliveryManager()
        dm.run()
    except rospy.ROSInterruptException:
        pass
