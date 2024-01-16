## Steps to run ROS Package:

1. Place our ROS2 galactic package (`arm_assm`) in your colcon workspace.

2. Run the `debug.launch.py` launch file. This will launch gazebo world with our robot, table, and box.

   ```bash
   $ ros2 launch arm_assm debug.launch.py
   ```

3. Run the inverse kinematics controller node.

   ```bash
   $ ros2 run arm_assm ik_controller.py
   ```

4. Run the teleop node.

   ```bash
   $ ros2 run arm_assm teleop.py
   ```

   (optional)

5. Run the node to output current end effector position w.r.t base_link

   ```bash
   $ ros2 run arm_assm ee_pos_output.py
   ```

6. Run the node to output current joint values of all joints.

   ```bash
   $ ros2 run arm_assm joint_state_listener.py
   ```

## Keyboard keys to control robot using teleop:

Key | Command
--- | -------
W   | Move Rover Forward
A   | Steer Rover Left
S   | Move Rover Reverse
D   | Steer Rover Right
Q   | Stop Robot
I   | Move Arm Forward
J   | Move Arm Left
K   | Move Arm Backward
L   | Move Arm Right
M   | Move Arm Upwards
N   | Move Arm Downward
H   | Set Arm in Home pose
R   | Set Arm in READY pose
C   | Vacuum Gripper ON
V   | Vacuum Gripper OFF
ESC | Quit Teleop

Note: In our teleop node, we are controlling the end effector position using the keys. In the background, inverse kinematics is used to compute the required joint angles to move the robot in the required trajectory. We are NOT manipulating the arm’s joints directly. Inverse Kinematics is doing it for us.

