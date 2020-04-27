import gym
from gym import spaces
import matplotlib.pyplot as plt
import os
import open3d as o3d

import pickle as pkl
import numpy as np
import time
from enum import Enum

from ormope.scene.scene import Scene
from ormope.scene.robot import Robot
from ormope.scene.obstacle import Obstacle
from ormope.envs.utils import obstacles_to_oriented_pointcloud


class ResultCode(Enum):
    SUCCESS = 0
    SELF_COLLISION = 1
    OBSTACLE_COLLISION = 2
    NON_TERMINAL = 3

class BaseEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, n_samples=120):
        self.scene = Scene()
        self.robot = Robot(os.path.join(os.getcwd(), 'ormope', 'assets', 'widowx', 'widowx_env.xml'))
        self.robot.load(self.scene)

        self.n_samples = n_samples

        self.obstacle_point_dim = 6
        self.obstacles_dim = self.n_samples * self.obstacle_point_dim

        # TODO: 2D-Dimension =? 2D goal pose
        self.goal_dim = 2

        # Theta representation: (sin theta_i, con theta_i) * num_joints + goal
        self.config_dim = 2 * self.robot.get_number_of_joints() + self.goal_dim

        self.action_space = spaces.Box(low=-1, high=1, shape=(self.robot.get_number_of_joints(), ), dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "observation": spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(self.obstacles_dim + self.config_dim,),
                    dtype=np.float32,
                ),
                "desired_goal": spaces.Box(
                    low=-1.0, high=1.0, shape=(self.goal_dim,), dtype=np.float32
                ),
                "achieved_goal": spaces.Box(
                    low=-1.0, high=1.0, shape=(self.goal_dim,), dtype=np.float32
                ),
                "representation_goal": spaces.Box(
                    low=-1.0, high=1.0, shape=(self.goal_dim,), dtype=np.float32,
                ),
            }
        )

        self.dict_reward = {"goal": 1, "collision": -1, "free": -0.1}
        self.truncate_penalty = 0.05

        self.obstacles = []

        self.obstacles_obs = []

        # TODO: Sample Goal?
        self.goal_pose = np.array([-0.5, 0.5])

        # Setup parameters
        self.action_step_size = 0.025
        self.segment_validity_step = 0.001
        self.goal_sensitivity = 0.04
        self.keep_alive_penalty = 0.01

    def seed(self, seed=None):
        np_random, seed = gym.utils.seeding.np_random(seed)
        self._seed = seed
        self._np_random = np_random
        return seed

    def reset(self):
        self.seed()
        #TODO: Load parameters from file
        ws = [
            Obstacle([0.08071720797526379, 0.01, 0.0258863013653452], [0.28880693169373045, 0, 0.13432395598779545], 1.3412717893000832),
            Obstacle([0.0215697165308143, 0.01, 0.011916695698156086], [-0.20279715759699013, 0, 0.2226465499274907], 0.3318132918029501),
            Obstacle([0.0115697165308143, 0.01, 0.11916695698156086], [-0.20279715759699013, 0, 0.2526465499274907], 0.3318132918029501),
            Obstacle([0.021596353356418477, 0.01, 0.025303435382963098], [-0.0932739818786471, 0, 0.34939100756120933], 0.7071332436658244)
        ]

        self.load_workspace(ws)

        self.scene.start_viz()

        return self.compute_observation()


    def load_workspace(self, workspace):
        for i in range(len(workspace)):
            obstacle = workspace[i]
            # Add body to the scene
            obstacle.load(self.scene, 'obstacle{}'.format(i))
            self.obstacles.append(obstacle)
            self.obstacles_obs = obstacles_to_oriented_pointcloud(self.obstacles, self.n_samples, self._np_random)

    def clean_workspace(self):
        while len(self.obstacles):
            obstacle = self.obstacles.pop()
            obstacle.remove(self.scene)

    def represent_obstacles(self):
        # TODO: We can easily change and add relative representation
        #obstacles_repr = obstacls_obs.copy()

        obstacles_obs = self.obstacles_obs.flatten()
        return obstacles_obs

    def represent_goal(self):
        # For relative position goal_repr = self.goal_pose - self.robot.get_link_pose(5)
        goal_repr = self.goal_pose

        return goal_repr

    def goal_reached(self):
        robot_pose = self.robot.get_link_pose(5)
        distance = np.linalg.norm(np.array(robot_pose) - np.array(self.goal_pose))
        return distance < self.goal_sensitivity

    def step(self, action):
        d_joints = action * self.action_step_size

        next_unbounded_joints = self.robot.get_joints() + d_joints
        next_joints = self.robot.truncate_joints(next_unbounded_joints)

        reward = 0.0

        self.robot.set_joints(next_joints)

        done = True

        if self.robot.check_self_collision():
            reward += self.dict_reward['collision']
            info = {'result': ResultCode.SELF_COLLISION}

        elif self.scene.check_collision(self.robot, self.obstacles):
            reward += self.dict_reward['collision']
            info = {'result': ResultCode.OBSTACLE_COLLISION}

        elif self.goal_reached():
            reward += self.dict_reward['goal']
            info = {'result': ResultCode.SUCCESS}

        else:
            reward += self.dict_reward['free']
            info = {'result': ResultCode.NON_TERMINAL}
            done = False

        if self.truncate_penalty > 0.0:
            reward -= self.truncate_penalty * np.linalg.norm(next_unbounded_joints - next_joints) / self.action_step_size

        observation = self.compute_observation()

        return observation, reward, done, info

    def compute_observation(self):
        observation = {
            "achieved_goal": self.robot.get_link_pose(5),
            "desired_goal": self.goal_pose,
            "representation_goal": self.represent_goal(),
            "observation": np.concatenate((self.represent_obstacles(), self.robot.get_representation())),
        }

        return observation

    def close(self):
        pass


if __name__ == "__main__":
    env = BaseEnv()
    obs = env.reset()
    print(obs)
    # plt.imshow(obs)
    # plt.show()
