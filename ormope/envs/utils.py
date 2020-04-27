import numpy as np
import open3d as o3d

def within_obstacle(obstacle, points):
    T_obs = obstacle.transformation_matrix
    points_hom = np.concatenate((points, np.ones((points.shape[0], 1))), axis=1)
    # set the points in the box referential
    T_obs_world = np.linalg.inv(T_obs)
    points_obs = points_hom.dot(T_obs_world.T)
    points_obs = points_obs[:, :3]
    hside = np.array(obstacle.sides)
    in_obs = np.logical_and(
        (points_obs > -hside).all(axis=1), (points_obs < hside).all(axis=1)
    )
    return in_obs

def obstacles_to_oriented_pointcloud(obstacles, n_samples, np_random):
    """
    generate a point cloud with normals from a set of boxes
    ensure that none of the points sampled on the boundary of one cube are within another cube
    """
    n_obstacles = len(obstacles)
    if n_obstacles == 0:
        return np.zeros((0, 0))

    obstacles_size = []
    for obstacle in obstacles:
        obstacles_size.append(list(np.array(obstacle.sides)/2))
    obstacles_size = np.array(obstacles_size)[:, 0]
    prop_sample = obstacles_size / obstacles_size.sum()

    samples = np.zeros((0, 6))
    for i, (obstacle, p) in enumerate(zip(obstacles, prop_sample)):
        points_obstacle, normals_obstacle = obstacle.to_oriented_pointcloud(
            4 * int(p * n_samples), np_random
        )
        in_other_obstacle = np.zeros(points_obstacle.shape[0], dtype=bool)
        for j, other_obstacle in enumerate(obstacles):
            if j != i:
                in_other_obstacle = np.logical_or(
                    in_other_obstacle, within_obstacle(other_obstacle, points_obstacle)
                )
        points_obstacle = points_obstacle[~in_other_obstacle]
        normals_obstacle = normals_obstacle[~in_other_obstacle]
        samples_obstacle = np.concatenate((points_obstacle, normals_obstacle), axis=1)
        samples_obstacle = samples_obstacle[: int(np.ceil(p * n_samples))]
        samples = np.concatenate((samples, samples_obstacle), axis=0)
    samples = samples[:n_samples]

    if samples.shape[0] < n_samples:
        n_missing_samples = n_samples - samples.shape[0]
        idx = np_random.randint(samples.shape[0], size=(n_missing_samples,))
        samples = np.concatenate((samples, samples[idx]), axis=0)

    return samples


def compute_transformation(position, theta):
    transformation_matrix = np.eye(4)
    translation = np.array(position)

    rotation_matrix = np.array([
        [np.cos(theta), 0.0, np.sin(theta)], [0.0, 1.0, 0.0], [-np.sin(theta), 0.0, np.cos(theta)]
    ])

    transformation_matrix[:3, -1] = translation
    transformation_matrix[:3, :3] = rotation_matrix

    return transformation_matrix


def print_obstacles(self, samples):
    points, normals = np.array(samples)[:, :3], np.array(samples)[:, 3:]
    pcd = o3d.geometry.PointCloud(o3d.open3d.utility.Vector3dVector(points))
    pcd.normals = o3d.open3d.utility.Vector3dVector(normals)

    vis = o3d.visualization.Visualizer()

    # TODO: Clean camera transform
    # Set camera
    camera_transform = np.eye(4)
    theta = -np.pi / 2
    rotation_matrix = np.array([
       [1.0, 0.0, 0.0], [0.0, np.cos(theta), -np.sin(theta)], [0.0, np.sin(theta), np.cos(theta)]
    ])
    camera_transform[:3, :3] = rotation_matrix
    camera_transform[:3, 3] = np.array([0.0, -1.0, 0.25])

    vis.create_window()
    pcd.paint_uniform_color([255, 0, 0])
    pcd.transform(camera_transform)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
