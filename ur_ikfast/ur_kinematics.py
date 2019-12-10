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

        if isinstance(joint_angles, np.ndarray):
            joint_angles = joint_angles.tolist()
        
        ee_pose = self.kinematics.forward(joint_angles)
        ee_pose = np.asarray(ee_pose).reshape(3, 4)

        if rotation_type == 'matrix':
            return ee_pose
        elif rotation_type == 'quaternion':
            return transformations.pose_quaternion_from_matrix(ee_pose)

    def inverse(self, ee_pose, all_solutions=False, q_guess=np.zeros(6)):
        joint_configs = self.kinematics.inverse(ee_pose.reshape(-1).tolist())
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
        print("ik failed")
        return None
    best_sol_ind = np.argmin(
        np.sum((weights * (valid_sols - np.array(q_guess)))**2, 1))
    return valid_sols[best_sol_ind]
