#!/usr/bin/env python3.8

from env_mvp import RobotArmEnv
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DQN

vec_env = DummyVecEnv([lambda: RobotArmEnv()])

model = PPO("MultiInputPolicy", vec_env, verbose=1, device="cpu")

# Train the agent and display a progress bar
model.learn(total_timesteps=10, progress_bar=True, log_interval=4)

# Save the agent
model.save("alice_mvp")

obs, info = vec_env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = vec_env.step(action)
    if terminated or truncated:
        obs, info = vec_env.reset()