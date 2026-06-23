#!/usr/bin/env python3
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
import numpy as np

env = DuckietownEnv(
    seed=1,
    map_name="udem1",
    draw_curve=False,
    draw_bbox=False,
    domain_rand=False,
    frame_skip=1,
    distortion=False,
)
obs = env.reset()

for i in range(100):
    action = np.array([0.3, 0.0])
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        env.reset()

print("Test completed!")
env.close()
