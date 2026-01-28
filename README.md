# lecture7_examples
Computing the inverse kinematics of a robot using KDL

# Build example
```bash
cd ~/workspace/examples_ws/
```
```bash
colcon build
```
# Run example
```bash
cd ~/workspace/examples_ws/
```
```bash
cd source install/setup.bash
```
Run previous example
```bash
ros2 launch scara_urdf_example scara.launch.py
```
In a new terminal window, source then run:
```bash
cd source install/setup.bash
ros2 run kdl_example kdl_example
```
Open a new terminal window
```bash
cd source install/setup.bash
rqt
```
Use the message publisher to publish message on the `/goal_position` topic