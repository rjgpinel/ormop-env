from openravepy import *
from ormope.envs.utils import compute_transformation

import numpy as np

class Obstacle:
    def __init__(self, sides, position, theta):
        self.sides = sides
        self.position = position
        self.theta = theta  # Y-axis rotation

        self.transformation_matrix = compute_transformation(position, theta)

    def load(self, scene, name):
        self.body = RaveCreateKinBody(scene.openrave_sim, '')
        self.body.SetName(name)
        self.body.InitFromBoxes(np.array([[0, 0, 0, self.sides[0], self.sides[1], self.sides[2]]]), True)
        scene.openrave_sim.Add(self.body, True)
        self.body.SetTransform(self.transformation_matrix)

    def remove(self, scene):
        if self.body is None:
            print("Body is not loaded.")
            return
        scene.openrave_sim.Remove(self.body)

    def to_oriented_pointcloud(self, n_samples, np_random):
        """
        sample points uniformly points on 2d squares and project them on one of the 4 faces
        then transform the points and normals to match the square pose
        """
        points = np_random.uniform(-1, 1, size=(n_samples, 3))
        points[:, 1] = 0
        normals = np.zeros((n_samples, 3))
        up_down_face = np_random.choice([-1, 1], p=[0.5, 0.5], size=(n_samples,))
        face = np_random.choice([0, 2], p=[0.5, 0.5], size=(n_samples,))
        points[np.arange(n_samples), face] = up_down_face
        normals[np.arange(n_samples), face] = up_down_face

        points *= np.array(self.sides)/2

        points = np.concatenate((points, np.ones((points.shape[0], 1))), axis=1)

        T_obs = np.array(self.transformation_matrix)

        points = points.dot(T_obs.T)
        points = points[:, :3]
        normals = normals.dot(T_obs[:3, :3].T)

        return points, normals
