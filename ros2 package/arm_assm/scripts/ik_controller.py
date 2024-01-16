#!/usr/bin/env python3

###################################################################
# This node controls the arm ee_position based on teleop command
###################################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from inverse_kinematics import InverseK
import sympy as smbl 
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String


class IKController(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_controller')
        self.jnt_pblshr = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.current_joint_angles = None
        self.jnt_state_sbscrbr = self.create_subscription(JointState,'joint_states',self.joint_states_callback,10  )
        self.jnt_state_sbscrbr  # so that unused variable warning can be skipped

        self.sbs = self.create_subscription(String, 'arm_joystick', self.joystick_input_callback, 10)

        self.IKSolver = InverseK()
        print (str(self.current_joint_angles))
        self.received_data= False   # to check if we got TF data 


    def joystick_input_callback(self, msg):
        if msg.data!="":
            self.move_robot_ik(msg.data + "_axis")


    def move_robot_ik(self, axis='-z_axis'):
        while rclpy.ok() and not self.received_data:
            print('Waiting for subscriber...')
            rclpy.spin_once(self)

        print ("Started Trajectory...")
        distance = 0.01 # every time button is pressed, the robot moves 1 cm in specified direction
        q_current_initial = smbl.Matrix(self.current_joint_angles)
        publisher_obj = self.jnt_pblshr
        self.IKSolver.move_robot(distance, axis, q_current_initial, publisher_obj)
        print ("Finished Trajectory...")

    # Ready pose is the pose where robot ee is facing downwards and robot as a whole is facing in front
    def go_to_ready_pose(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.56,-1.49,0.5,4.68,0.03,0.0,0.0]
        self.jnt_pblshr.publish(msg)

    def joint_states_callback(self, msg):
        print("================================")
        print("LIVE JOINT VALUES")
        angle_dict ={}
        angles =[]
        for i in range(len(msg.name)):
            joint_name = msg.name[i]
            joint_position = msg.position[i]
            angles.append(joint_position)
            angle_dict[joint_name] = joint_position

        self.current_joint_angles=[round(angle_dict["joint_1"],3)], [round(angle_dict["connection_1"],3)], [round(angle_dict["joint_2"],3)], [round(angle_dict["joint_3"],3)], [round(angle_dict["joint_4"],3)], [round(angle_dict["joint_5"],3)]
        print (str(self.current_joint_angles))
        self.received_data = True
        print("================================")


def main(args=None):
    rclpy.init(args=args)
    ik_joystick = IKController()
    rclpy.spin(ik_joystick)
    ik_joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
