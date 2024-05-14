#!/usr/bin/env python

from env import RobotArmEnv
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

vec_env = DummyVecEnv([lambda: RobotArmEnv()])
model = PPO("MlpPolicy", vec_env, verbose=1)

# Train the agent and display a progress bar
model.learn(total_timesteps=100, progress_bar=True)

# Save the agent
model.save("alice_mvp")

# Run trained agent
obs = vec_env.reset()
for i in range(10):
    action, _state = model.predict(obs, deterministic=True)
    print(f"Predicted action: {action}")  # Print the predicted action
    obs, reward, done, info = vec_env.step(action)