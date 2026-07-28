# KDL_example
Computing the inverse kinematics of a robot using KDL. Builds on the previous example: [URDF_example](https://github.com/Vanderbilt-Applied-Robotics-Lab/URDF_example)

## Downloading Code
1. Navigate to examples workspace source folder: `cd ~/workspaces/examples_ws/src`
2. Download code: `https://github.com/Vanderbilt-Applied-Robotics-Lab/KDL_example.git`

## Compiling Code
1. Navigate to examples workspace: `cd ~/workspaces/examples_ws`
2. Compile the code: `colcon build`

## Running Code
1. Navigate to examples workspace: `cd ~/workspaces/examples_ws`
2. Source the code: `source install/setup.bash`
3. Run previous example: `ros2 launch scara_urdf_example scara.launch.py`
4. Open a new terminal window
5. Source the code: `source install/setup.bash`
6. Run the example: `ros2 run kdl_example kdl_example`
7. Open a new terminal window
8. Use RQT to publish to the `/goal_position` topic (see the instructions on the lecture slides): `rqt`