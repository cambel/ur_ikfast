import numpy as np
import ur3e_ikfast
import ur5_ikfast

# Initialize kinematics for UR5 robot arm
ur3e_kin = ur3e_ikfast.PyKinematics()
ur5_kin = ur5_ikfast.PyKinematics()
n_joints = ur3e_kin.getDOF()

joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians

# Test forward kinematics: get end effector pose from joint angles
print("\nTesting forward kinematics:\n")
print("Joint angles:")
print(joint_angles)
ee_pose = ur3e_kin.forward(joint_angles)
ee_pose = np.asarray(ee_pose).reshape(3, 4)  # 3x4 rigid transformation matrix
print("\nEnd effector pose:")
print(ee_pose)
print("\n-----------------------------")

# Test inverse kinematics: get joint angles from end effector pose
print("\nTesting inverse kinematics:\n")
joint_configs = ur3e_kin.inverse(ee_pose.reshape(-1).tolist())
n_solutions = int(len(joint_configs)/n_joints)
print("%d solutions found:" % (n_solutions))
joint_configs = np.asarray(joint_configs).reshape(n_solutions, n_joints)
for joint_config in joint_configs:
    print(joint_config)

# Check cycle-consistency of forward and inverse kinematics
assert(np.any([np.sum(np.abs(joint_config-np.asarray(joint_angles))) < 1e-4 for joint_config in joint_configs]))
print("\nTest passed!")

print("\nTesting forward kinematics UR5:\n")
print("Joint angles:")
print(joint_angles)
ee_pose = ur5_kin.forward(joint_angles)
ee_pose = np.asarray(ee_pose).reshape(3, 4)  # 3x4 rigid transformation matrix
print("\nEnd effector pose:")
print(ee_pose)
print("\n-----------------------------")