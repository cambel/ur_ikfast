from ur_ikfast import ur_kinematics
import numpy as np

robot_name = 'ur10'
arm = ur_kinematics.URKinematics(robot_name)
ikfast_res = 0
total = 10000

for i in range(total):
    joint_angles = np.random.uniform(-1*np.pi, 1*np.pi, size=6)
    pose = arm.forward(joint_angles)
    ik_solution = arm.inverse(pose, False, q_guess=joint_angles)
    if ik_solution is not None:
        ikfast_res += 1 if np.allclose(joint_angles, ik_solution, rtol=0.01) else 0
print("IKFAST success rate %s of %s" % (ikfast_res, total))
print("percentage %.1f", ikfast_res/float(total)*100.)
