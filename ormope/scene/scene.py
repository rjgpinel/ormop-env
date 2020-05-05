from openravepy import *
import numpy as np
import time

class Scene:
    def __init__(self):
        self.openrave_sim = Environment()
        self.openrave_sim.StopSimulation()

    def start_viz(self):
        if self.openrave_sim.GetViewer() is None:
            self.openrave_sim.SetViewer('qtcoin')

        # Set camera
        camera_transform = np.eye(4)
        theta = -np.pi / 2
        rotation_matrix = np.array([
           [1.0, 0.0, 0.0], [0.0, np.cos(theta), -np.sin(theta)], [0.0, np.sin(theta), np.cos(theta)]
        ])
        camera_transform[:3, :3] = rotation_matrix
        camera_transform[:3, 3] = np.array([0.0, -1.0, 0.25])
        time.sleep(1)
        viewer = self.openrave_sim.GetViewer()
        viewer.SetCamera(camera_transform)

    def check_collision(self, robot, obstacles):
        if obstacles is not None:
            for obstacle in obstacles:
                if self.openrave_sim.CheckCollision(robot.controller, obstacle.body):
                    return True
        return False

    def destroy(self):
        RaveDestroy()
