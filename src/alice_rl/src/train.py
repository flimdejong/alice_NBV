#! /usr/bin/env python

from stable_baselines3 import PPO
from env import RobotArmEnv

# Create an instance of your custom Gym environment
env = RobotArmEnv()

# Create an instance of the PPO algorithm
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=10000)

# Save the trained model
model.save("alice_mvp")

# Evaluate the trained model
mean_reward, _ = model.evaluate(env, num_episodes=10)
print(f"Mean reward: {mean_reward}")

