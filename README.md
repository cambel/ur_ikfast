# ur_ikfast

IKFast libraries for UR3e, UR3 and UR5.

Inspired by https://github.com/andyzeng/ikfastpy

Easy to install and use:
1. clone
2. install dependencies

`sudo apt-get install libblas-dev liblapack-dev`

`pip install --user numpy Cython`

3. install using pip </br>

`$ git clone https://github.com/cambel/ur_ikfast.git`</br>
`$ cd ur_ikfast`</br>
`$ pip install -e .`</br>

It takes a few minutes to compile the ikfast libraries.

#### Example
```python
from ur_ikfast import ur_kinematics

ur3e_arm = ur_kinematics.URKinematics('ur3e')

joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
print("joint angles", joint_angles)

pose_quat = ur3e_arm.forward(joint_angles)
pose_matrix = ur3e_arm.forward(joint_angles, 'matrix')

print("forward() quaternion \n", pose_quat)
print("forward() matrix \n", pose_matrix)

# print("inverse() all", ur3e_arm.inverse(pose_quat, True))
print("inverse() one from quat", ur3e_arm.inverse(pose_quat, False, q_guess=joint_angles))

print("inverse() one from matrix", ur3e_arm.inverse(pose_matrix, False, q_guess=joint_angles))
```

For a new robot just create the ikfast database (.cpp) following one of these tutorials:
- http://docs.ros.org/kinetic/api/framefab_irb6600_support/html/doc/ikfast_tutorial.html
- http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html
