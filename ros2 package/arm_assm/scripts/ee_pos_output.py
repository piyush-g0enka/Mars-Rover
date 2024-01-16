#!/usr/bin/env python3

###########################################################################
# This node prints the EE position at a freuency of 1/3 Hz
# This node is used to validate FK and IK
###########################################################################

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer as Bfr
from tf2_ros import TransformException as TrExc
from tf2_ros.transform_listener import TransformListener as TL

class EEPosOutput(Node):

    def __init__(self):
        super().__init__('ee_position_listener')

        self.tfbffr = Bfr()         # TF Buffer
        self.tflstnr = TL(self.tfbffr, self)    # TF Listener

        # Output EE position every 3 seconds
        self.tmr = self.create_timer(3.0, self.cb_tmr)

    def cb_tmr(self):

        child_frame = 'motor_5'  
        parent_frame = 'base_link'

        try:

            current_time = rclpy.time.Time()
            t = self.tfbffr.lookup_transform(
                parent_frame,
                child_frame,
                current_time)
            print("================================")
            print ("EE position")
            print("x: "+ str(round(t._transform._translation.x,2)))
            print("y: "+ str(round(t._transform._translation.y,2)))
            print("z: "+ str(round(t._transform._translation.z,2)))
            print("================================")
           
        except TrExc as q:
            print("EE positions not available")
            return



def main():
    rclpy.init()
    node = EEPosOutput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()



main()