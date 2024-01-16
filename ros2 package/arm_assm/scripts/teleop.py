#!/usr/bin/env python3

###############################################################
# This node takes KBD input and send out the reuired messages
# in various topics to control the robot
###############################################################

import rclpy
import termios as trmnl
from sensor_msgs.msg import JointState
import select
import sys as s
from std_msgs.msg import Float64MultiArray
import tty as serial_comms
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
#################################################################################################################

# step sizes
LINEAR_SPEED_DELTA = 8
STEER_ANGLE_DELTA = 0.08

#################################################################################################################

########################################################
# Node for teleop
########################################################

class TeleOpNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        self.jnt_st_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.whl_vel_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_pos_publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.string_publisher = self.publisher_ = self.create_publisher(String, '/arm_joystick', 10)
        self.vacuum_gripper_client = self.create_client(SetBool, '/vacuum_gripper/custom_switch')
        self.settings = trmnl.tcgetattr(s.stdin)
        self.arm_v = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.publish_type=""

    ########################################################
    # Function to get input from keyboard
    ########################################################

    def getKbdInput(self):
        serial_comms.setraw(s.stdin.fileno())
        rlist, _, _ = select.select([s.stdin], [], [], 0.1)
        if rlist:
            key = s.stdin.read(1)
        else:
            key = ''

        trmnl.tcsetattr(s.stdin, trmnl.TCSADRAIN, self.settings)
        return key


    ########################################################
    # Function to convery key inputs to velocity and
    # position messages
    ########################################################

    def run_keyboard_control(self):

        jnt_pos = Float64MultiArray()
        whl_vel = Float64MultiArray()
        frwd_vel=0.0
        str_angle=0.0

        while True:
            button_pressed = self.getKbdInput()
            if button_pressed is not None:
                if button_pressed == '\x1b':  # ESC
                    break
                elif button_pressed == 'q':  # Quit
                    frwd_vel=0.0
                    str_angle=0.0
                    self.publish_type="joint"
                elif button_pressed == 'w':  # FWD
                    frwd_vel += LINEAR_SPEED_DELTA
                    self.publish_type="joint"
                elif button_pressed == 'a':  # LFT
                    str_angle += STEER_ANGLE_DELTA
                    self.publish_type="joint"
                elif button_pressed == 'd':  # RGHT
                    str_angle -= STEER_ANGLE_DELTA
                    self.publish_type="joint"
                elif button_pressed == 's':  # RVS
                    frwd_vel -= LINEAR_SPEED_DELTA
                    self.publish_type="joint"

                # limit for steering angle to run smoothly
                if str_angle>2.0:
                        str_angle=2.0
                if str_angle<-2.0:
                    str_angle=-2.0

                if button_pressed == 'h':  # go to Home pose
                    self.arm_v = [0.0,0.0,0.0,0.0,0.0,0.0]
                    self.publish_type="joint"
                elif button_pressed == 'r':  # go to READY pose
                    self.arm_v = [0.0, 0.56,-1.49,0.5,4.68,0.03]
                    self.publish_type="joint"

                
                arm_direction=""
                if button_pressed == 'i':  # FWD ARM
                    arm_direction="x"
                    self.publish_type="string"
                elif button_pressed == 'j':  # LFT ARM
                    arm_direction="y"
                    self.publish_type="string"
                elif button_pressed == 'l':  # RGHT ARM
                    arm_direction="-y"
                    self.publish_type="string"
                elif button_pressed == 'k':  # BWD ARM
                    arm_direction="-x" 
                    self.publish_type="string"               
                elif button_pressed == 'm':  # UP ARM
                    arm_direction="z"
                    self.publish_type="string"
                elif button_pressed == 'n':  # DWN ARM
                    arm_direction="-z"  
                    self.publish_type="string"

                if button_pressed == 'c':   # Vacuum Gripper ON

                    request = SetBool.Request()
                    request.data = True
                    future = self.vacuum_gripper_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future)

                    if future.result() is not None:
                        response = future.result()
                        print(str(response.success))
                    else:
                        print('try again....')
                    self.publish_type="service"

                elif button_pressed == 'v':     # Vacuum Gripper OFF
                    request = SetBool.Request()
                    request.data = False
                    future = self.vacuum_gripper_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future)

                    if future.result() is not None:
                        response = future.result()
                        print(str(response.success))
                    else:
                        print('try again....')
                    self.publish_type="service"


                if self.publish_type=="joint":
                    whl_vel.data = [frwd_vel,frwd_vel,-frwd_vel]
                    jnt_pos.data = [self.arm_v[0],self.arm_v[1],self.arm_v[2],self.arm_v[3],self.arm_v[4],self.arm_v[5],str_angle,str_angle]

                    self.joint_pos_publisher.publish(jnt_pos)
                    self.whl_vel_publisher.publish(whl_vel)

                    print("----------------------")
                    print("Linear Velocity",frwd_vel)
                    print("Steer Angle",str_angle)
                    print("----------------------")

                elif self.publish_type=="string":
                    msg = String()
                    msg.data = arm_direction
                    self.string_publisher.publish(msg)
                    print ("Arm direction--> " + str(msg))


#################################################################################################################

########################################################
# main function invokes our Node
########################################################

def main(args=None):
    rclpy.init(args=args)
    node = TeleOpNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()


#################################################################################################################


if __name__ == '__main__':
    main()