#!/usr/bin/env python3

########################################################################
# This node is used to print the current joint values of the robot
########################################################################

import rclpy
from sensor_msgs.msg import JointState

def cb_jnts(msg):
    print("Received JointStates:")
    for i in range(len(msg.name)):
        j_name = msg.name[i]
        j_position = msg.position[i]
        print(f"{j_name}: {j_position}")
        

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('joint_state_listener')

    joint_sub= node.create_subscription(JointState, 'joint_states', cb_jnts ,10 )
    joint_sub #to prevent warning of unused variable

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
