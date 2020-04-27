import numpy as np


class Robot:

    def __init__(self, robot_description):

        self.robot_description = robot_description

        self.controller = None
        self.links = None
        self.joint_safety = 0.0001


    def load(self, scene):
        scene.openrave_sim.Load(self.robot_description)
        self.controller = scene.openrave_sim.GetRobots()[0]
        self.controller.SetActiveDOFs(range(1, 5))
        self.links = self.controller.GetLinks()

        # Set default color
        color = np.array([33, 213, 237])
        for link in self.links:
            for geom in link.GetGeometries():
                geom.SetDiffuseColor(color)

        self.links = self.controller.GetLinks()

    def get_number_of_joints(self):
        return self.controller.GetDOF()

    def get_joints(self):
        return self.controller.GetDOFValues()

    def get_joint_bounds(self):
        return self.controller.GetDOFLimits()

    def get_random_joints(self, fixed_positions_dictionary=None):
        joint_bounds = self.get_joint_bounds()
        result = []
        for i in range(self.get_number_of_joints()):
            if fixed_positions_dictionary is not None and i in fixed_positions_dictionary:
                result.append(fixed_positions_dictionary[i])
            else:
                result.append(random.uniform(joint_bounds[0][i], joint_bounds[1][i]))
        result = self.truncate_joints(result)
        return tuple(result)

    def truncate_joints(self, joints):

        bounds = self.get_joint_bounds()
        res = list(joints)
        for i, j in enumerate(joints):
            lower = bounds[0][i] + self.joint_safety
            res[i] = max(res[i], lower)
            upper = bounds[1][i] - self.joint_safety
            res[i] = min(res[i], upper)
        return tuple(res)

    def is_valid(self, joints):
        self.robot.SetDOFValues(joints, [0, 1, 2, 3, 4])
        res = not self.controller.CheckSelfCollision()
        if self.objects is not None:
            for item in self.objects:
                res = res and not self.env.CheckCollision(self.robot, item)
        return res

    def plan(self, start_joints, goal_joints, max_planner_iterations):
        with self.env:
            if not self.is_valid(start_joints) or not self.is_valid(goal_joints):
                return None
            self.controller.SetDOFValues(start_joints, [0, 1, 2, 3, 4])
            manipprob = interfaces.BaseManipulation(self.robot)  # create the interface for basic manipulation programs
            try:
                items_per_trajectory_step = 10
                active_joints = self.controller.GetActiveDOF()
                # call motion planner with goal joint angles
                traj = manipprob.MoveActiveJoints(goal=goal_joints[1:], execute=False, outputtrajobj=True, maxtries=1,
                                                  maxiter=max_planner_iterations)
                # found plan, if not an exception is thrown and caught below
                traj = list(traj.GetWaypoints(0, traj.GetNumWaypoints()))
                assert len(traj) % items_per_trajectory_step == 0
                # take only the joints values and add the 0 joint.
                traj = [[0.0] + traj[x:x + items_per_trajectory_step][:active_joints] for x in
                        xrange(0, len(traj), items_per_trajectory_step)]
                # assert validity
                if self.get_last_valid_in_trajectory(traj) != traj[-1]:
                    return None
                # plan found and validated!
                return traj
            except Exception, e:
                print str(e)
                return None

    def get_links_poses(self):
        poses = self.controller.GetLinkTransformations()
        result = np.array([tuple(poses[i][[0, 1, 2], -1]) for i in range(self.get_number_of_joints())])
        return result

    def get_link_pose(self, link_id):
        #TODO: Check link_id is valid
        T = self.controller.GetLinks()[link_id].GetTransform()
        return T[[0, 2], -1]

    def set_joints(self, joints):
        self.controller.SetDOFValues(joints, list(range(self.get_number_of_joints())))

    def check_self_collision(self):
        return self.controller.CheckSelfCollision()

    def get_representation(self):
        #TODO: Compute Theta representation
        return self.get_links_poses().flatten()
