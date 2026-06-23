#!/usr/bin/env python3
import rospy
import cv2
import json
import os
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped, LanePose
from std_msgs.msg import String

# Estructura basada en ChallengeSolver oficial
TILE_SIZE = 0.585
QR_DETECTION_RADIUS = 0.45

# Posiciones oficiales de los QR (basadas en duckie_day_challenge.py)
QR_POSITIONS = {
    "QR-00": [1.5, 5.5], "QR-01": [5.5, 5.5], "QR-02": [6.5, 1.5],
    "QR-03": [2.5, 1.5], "QR-05": [6.5, 3.5], "QR-06": [3.5, 4.5],
    "QR-07": [2.5, 5.5], "QR-08": [4.5, 4.5],
}

def load_qr_config(qr_id, qr_dir='/data/qr_codes'):
    path = os.path.join(qr_dir, f"{qr_id}.json")
    try:
        with open(path) as f: return json.load(f)
    except: return {"qr_id": qr_id, "tiempo_detencion": 3}

class DuckieDayNode:
    def __init__(self):
        rospy.init_node('duckie_day_node', anonymous=True)
        self.state = "navegar"
        self.wait_timer = 0.0
        self.visited_qrs = set()
        self.current_delivery_idx = 0
        
        self.pub_wheels = rospy.Publisher('~wheels_cmd', Twist2DStamped, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber('~lane_pose', LanePose, self.lane_pose_callback, queue_size=1)
        self.latest_lane_pose = None
        self.cur_pos = [0,0] # Esto debería venir de env.cur_pos en simulación real

    def lane_pose_callback(self, msg):
        self.latest_lane_pose = msg
        # Nota: En un nodo ROS real, la posición se obtiene de TF o odometría
        
    def check_near_qr(self, pos):
        for qr_id, qr_pos in QR_POSITIONS.items():
            if qr_id in self.visited_qrs: continue
            dist = math.sqrt((pos[0] - qr_pos[0]*TILE_SIZE)**2 + (pos[1] - qr_pos[1]*TILE_SIZE)**2)
            if dist < QR_DETECTION_RADIUS: return qr_id
        return None

    def publish_wheels_cmd(self, v, omega):
        msg = Twist2DStamped()
        msg.v, msg.omega = v, omega
        self.pub_wheels.publish(msg)

    def image_callback(self, msg):
        # Lógica de estados principal
        if self.wait_timer > 0:
            self.wait_timer -= 0.1 # Simplificado
            self.publish_wheels_cmd(0.0, 0.0)
            return

        # Detección posicional (No por cámara)
        qr_id = self.check_near_qr(self.cur_pos)
        if qr_id:
            self.process_qr(qr_id)
            return

        # Navegación básica PID (Lane Following)
        if self.latest_lane_pose:
            # PID simplificado
            v = 0.3
            omega = - (self.latest_lane_pose.phi + 2.0 * self.latest_lane_pose.d)
            self.publish_wheels_cmd(v, omega)

    def process_qr(self, qr_id):
        self.visited_qrs.add(qr_id)
        cfg = load_qr_config(qr_id)
        rospy.loginfo(f"📍 Acción QR: {cfg}")
        self.wait_timer = cfg.get("tiempo_detencion", 3)
        # Aquí ejecutar luces según cfg["tipo_luz"]...
