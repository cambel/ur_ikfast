from ur_ikfast import ur_kinematics

import numpy as np

robot_name = 'ur5e'
arm = ur_kinematics.URKinematics(robot_name)
ikfast_res = 0
tracik_res = 0
total = 1000

for i in range(total):
    joint_angles = np.random.uniform(-2*np.pi, 2*np.pi, size=6)
    pose = arm.forward(joint_angles)
    ik_solution = arm.inverse(pose, False, q_guess=joint_angles)
    if ik_solution is not None:
        ikfast_res += 1 if np.allclose(joint_angles, ik_solution, rtol=0.01) else 0
print("IKFAST rate %s of %s" % (ikfast_res, total))
print("percentage", ikfast_res/float(total))
