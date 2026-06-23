#!/usr/bin/env python3
import sys
import argparse
import numpy as np
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.simulator import Simulator

parser = argparse.ArgumentParser()
parser.add_argument('--env-name', default='Duckietown-udem1-v0')
parser.add_argument('--map-name', default='udem1')
parser.add_argument('--distortion', default=False, action='store_true')
parser.add_argument('--draw-curve', action='store_true')
parser.add_argument('--draw-bbox', action='store_true')
parser.add_argument('--domain-rand', action='store_true')
parser.add_argument('--frame-skip', default=1, type=int)
parser.add_argument('--seed', default=1, type=int)
args = parser.parse_args()

env = DuckietownEnv(
    seed=args.seed,
    map_name=args.map_name,
    draw_curve=args.draw_curve,
    draw_bbox=args.draw_bbox,
    domain_rand=args.domain_rand,
    frame_skip=args.frame_skip,
    distortion=args.distortion,
)
obs = env.reset()
env.render()

total_reward = 0
step = 0
while True:
    lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
    distance_to_road_center = lane_pose.dist
    angle_from_road_center = lane_pose.angle_rad

    kp_angle = 10.0
    kp_distance = 5.0
    steering = kp_angle * angle_from_road_center + kp_distance * distance_to_road_center
    steering = np.clip(steering, -1, 1)

    speed = 0.3

    action = np.array([speed, steering])
    obs, reward, done, info = env.step(action)
    total_reward += reward
    step += 1
    print(f"step={step} reward={reward:.3f} total={total_reward:.3f}")
    env.render()
    if done:
        print("Done! Resetting...")
        obs = env.reset()
        total_reward = 0
        step = 0
