import math
import numpy as np

def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True
    """
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

def pose_quaternion_from_matrix(matrix):
    """Return translation + quaternion(x,y,z,w)
    """
    if matrix.shape == (3, 4):
        matrix = np.concatenate((matrix, [[0, 0, 0, 1]]), axis=0)

    pose = matrix[:3, 3]
    quat = quaternion_from_matrix(matrix)
    return np.concatenate((pose, quat), axis=0)

class URKinematics():

    def __init__(self, robot_name):
        if robot_name == 'ur3':
            import ur3_ikfast as ur_ikfast
        elif robot_name == 'ur3e':
            import ur3e_ikfast as ur_ikfast
        elif robot_name == 'ur5':
            import ur5_ikfast as ur_ikfast
        elif robot_name == 'ur5e':
            import ur5e_ikfast as ur_ikfast
        elif robot_name == 'ur10':
            import ur10_ikfast as ur_ikfast
        elif robot_name == 'ur10e':
            import ur10e_ikfast as ur_ikfast
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
            return pose_quaternion_from_matrix(ee_pose)

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
