import numpy as np
from ur_control import transformations


class URKinematics():

    def __init__(self, robot_name):
        if robot_name == 'ur3':
            import ur3_ikfast as ur_ikfast
        elif robot_name == 'ur3e':
            import ur3e_ikfast as ur_ikfast
        elif robot_name == 'ur5':
            import ur5_ikfast as ur_ikfast
        else:
            raise Exception("Unsupported robot")

        self.kinematics = ur_ikfast.PyKinematics()
        self.n_joints = self.kinematics.getDOF()

    def forward(self, joint_angles, rotation_type='quaternion'):
        """
            Compute robot's forward kinematics for the specified robot
            joint_angles: list
            rotation_type: 'quaternion' or 'matrix'
            :return: if 'quaternion' then return a list of [x, y, z, w. qx, qy, qz]
                     if 'matrix' then a list of 12 values the 3x3 rotation matrix and 
                     the 3 translational values
        """
        if isinstance(joint_angles, np.ndarray):
            joint_angles = joint_angles.tolist()

        ee_pose = self.kinematics.forward(joint_angles)
        ee_pose = np.asarray(ee_pose).reshape(3, 4)

        if rotation_type == 'matrix':
            return ee_pose
        elif rotation_type == 'quaternion':
            return transformations.pose_quaternion_from_matrix(ee_pose)

    def inverse(self, ee_pose, all_solutions=False, q_guess=np.zeros(6)):
        """ Compute robot's inverse kinematics for the specified robot
            ee_pose: list of 7 if quaternion [x, y, z, w, qx, qy, qz]
                     list of 12 if rotation matrix + translational values
            all_solutions: whether to return all the solutions found or just the best one
            q_guess:  if just one solution is request, this set of joint values will be use
                      to find the closest solution to this
            :return: list of joint angles
                     list of best joint angles if found
                     q_guess if no solution is found
        """
        pose = None
        if len(ee_pose) == 7:
            rot = np.roll(ee_pose[3:], 1)
            pose = np.concatenate((ee_pose[:3], rot), axis=0)
        else:
            pose = ee_pose
        joint_configs = self.kinematics.inverse(pose.reshape(-1).tolist())
        n_solutions = int(len(joint_configs)/self.n_joints)
        joint_configs = np.asarray(joint_configs).reshape(n_solutions, self.n_joints)

        if all_solutions:
            return joint_configs

        return best_ik_sol(joint_configs, q_guess)


def best_ik_sol(sols, q_guess, weights=np.ones(6)):
    """ Get best IK solution """
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6) * 9999.
        for i in range(6):
            for add_ang in [-2. * np.pi, 0, 2. * np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2. * np.pi
                        and abs(test_ang - q_guess[i]) <
                        abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if not valid_sols:
        return None
    best_sol_ind = np.argmin(
        np.sum((weights * (valid_sols - np.array(q_guess)))**2, 1))
    return valid_sols[best_sol_ind]
