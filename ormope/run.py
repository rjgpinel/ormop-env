import time
import numpy as np

from ormope.envs.base_env import BaseEnv

class RandomAgent:
    def __init__(self, action_dim):
        self.action_dim = action_dim

    def get_action(self, obs):
        """
        TanhGaussian, similar to SAC exploration
        """
        action = np.random.normal(size=(self.action_dim,))
        action = np.tanh(action)
        return action


if __name__ == "__main__":
    env = BaseEnv()
    obs = env.reset()

    agent = RandomAgent(5)

    horizon = 100

    print(obs)
    for i in range(horizon):
        action = agent.get_action(obs)
        obs, reward, done, info = env.step(action)
        print(obs)
        time.sleep(0.5)
        if done:
            exit()
