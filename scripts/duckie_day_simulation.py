#!/usr/bin/env python3
import json
import os
import sys
import time
import numpy as np

ENV_NAME = os.environ.get('DUCKIE_ENV', 'Duckietown-udem1-v0')
DELIVERIES_PATH = os.environ.get('DELIVERIES_PATH', '/data/deliveries.json')
MAP_NAME = os.environ.get('MAP_NAME', 'duckie_day_2026')

import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.simulator import Simulator


def load_deliveries(path):
    default = [
        {"id": "QR001", "station": "P1", "feed": "Alimento Salmón Iniciador A", "kg": 50.0, "priority": 1},
        {"id": "QR002", "station": "P2", "feed": "Alimento Salmón Crecimiento B", "kg": 30.0, "priority": 2},
        {"id": "QR003", "station": "P3", "feed": "Alimento Salmón Engorda C", "kg": 40.0, "priority": 3},
    ]
    try:
        with open(path) as f:
            data = json.load(f)
            deliveries = data.get('deliveries', data)
            return sorted(deliveries, key=lambda item: item.get('priority', 999))
    except Exception:
        return default


class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt=0.1):
        self.integral += error * dt
        derivative = (error - self.last_error) / max(dt, 0.01)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return np.clip(output, -1.0, 1.0)


class DuckieDaySolver:
    def __init__(self, deliveries):
        self.deliveries = sorted(deliveries, key=lambda item: item.get('priority', 999))
        self.current_idx = 0
        self.completed = [False] * len(deliveries)
        self.pid_angle = PIDController(kp=8.0, ki=0.0, kd=0.5)
        self.pid_distance = PIDController(kp=4.0, ki=0.0, kd=0.2)
        self.base_speed = 0.3
        self.delivery_mode = False
        self.delivery_timer = 0
        self.delivery_duration = 3.0
        self.total_reward = 0.0
        self.collisions = 0
        self.track_exits = 0
        self.deliveries_made = 0
        # Simular detección QR cada ~120 pasos (estaciones)
        self.qr_step_interval = 120
        self.last_qr_step = 0

    def get_action(self, env, dt=0.1):
        if self.delivery_mode:
            self.delivery_timer += dt
            if self.delivery_timer >= self.delivery_duration:
                self.delivery_mode = False
                self.delivery_timer = 0
                self.completed[self.current_idx] = True
                d = self.deliveries[self.current_idx]
                print(f"[DELIVERY] Station {d['station']}: {d['feed']} ({d['kg']}kg) - COMPLETE!")
                self.deliveries_made += 1
                self.current_idx += 1
                if self.current_idx >= len(self.deliveries):
                    print("[MISSION] All deliveries complete!")
                    return np.array([0.0, 0.0])
                self.pid_angle.reset()
                self.pid_distance.reset()
            return np.array([0.0, 0.0])

        lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
        dist = lane_pose.dist
        angle = lane_pose.angle_rad

        steer_angle = self.pid_angle.compute(angle, dt)
        steer_dist = self.pid_distance.compute(dist, dt)
        steering = np.clip(steer_angle + steer_dist, -1.0, 1.0)

        speed = self.base_speed * max(0.1, 1.0 - abs(dist) * 0.5)
        speed = np.clip(speed, 0.0, self.base_speed)

        return np.array([speed, steering])

    def on_collision(self):
        self.collisions += 1
        print(f"[COLLISION #{self.collisions}] +10s penalty")
        return -10.0

    def on_track_exit(self):
        self.track_exits += 1
        print(f"[TRACK EXIT #{self.track_exits}] +15s penalty")
        return -15.0

    def start_delivery(self):
        if self.current_idx < len(self.deliveries):
            d = self.deliveries[self.current_idx]
            print(f"\n[DELIVERY] Station {d['station']} - {d['feed']} ({d['kg']}kg)")
            self.delivery_mode = True
            self.delivery_timer = 0


def main():
    deliveries = load_deliveries(DELIVERIES_PATH)
    print("=" * 60)
    print("DUCKIE DAY 2026 - SALMON FEED DELIVERY")
    print("=" * 60)
    print(f"\nDeliveries ({len(deliveries)}):")
    for d in deliveries:
        print(f"  {d['id']}: Station {d['station']} -> {d['feed']} ({d['kg']}kg)")
    print(f"\nMap: {ENV_NAME}")
    print("Controls: ESC=exit, BACKSPACE=reset\n")

    env = DuckietownEnv(
        seed=1,
        map_name=MAP_NAME,
        draw_curve=False,
        draw_bbox=True,
        domain_rand=False,
        frame_skip=1,
        distortion=False,
    )

    solver = DuckieDaySolver(deliveries)
    obs = env.reset()
    env.render()

    step = 0
    start_time = time.time()
    max_steps = 3000

    try:
        while step < max_steps:
            dt = 1.0 / env.unwrapped.frame_rate
            action = solver.get_action(env, dt)
            obs, reward, done, info = env.step(action)
            step += 1
            solver.total_reward += reward

            collision = info.get('collision', False)
            if collision:
                solver.on_collision()

            if step % 10 == 0:
                lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
                print(f"  step={step} reward={solver.total_reward:.2f} "
                      f"dist={lane_pose.dist:.3f} angle={lane_pose.angle_rad:.3f} "
                      f"deliveries={solver.deliveries_made}/{len(deliveries)}")

            env.render()

            if (solver.current_idx < len(deliveries)
                    and not solver.delivery_mode
                    and step - solver.last_qr_step >= solver.qr_step_interval):
                solver.start_delivery()
                solver.last_qr_step = step

            if done:
                print("RESET")
                obs = env.reset()
                solver.total_reward = 0
                step = 0

            if solver.current_idx >= len(deliveries):
                elapsed = time.time() - start_time
                print("\n" + "=" * 60)
                print("MISSION COMPLETE!")
                print(f"Time: {elapsed:.1f}s")
                print(f"Deliveries: {solver.deliveries_made}/{len(deliveries)}")
                print(f"Collisions: {solver.collisions}")
                print(f"Track exits: {solver.track_exits}")
                print(f"Total reward: {solver.total_reward:.2f}")
                print("=" * 60)
                time.sleep(5)
                break

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    finally:
        env.close()


if __name__ == '__main__':
    main()
