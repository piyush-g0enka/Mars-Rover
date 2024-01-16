#!/usr/bin/env python3

##################################################################
# This scripts computes the IK required to move the arm
# This script is used in ik_controller node
##################################################################
import sympy as smbl
from std_msgs.msg import Float64MultiArray
import numpy as nmbr

class InverseK:
    def __init__(self):
        
        # Define joints
        self.theta_1, self.theta_2, self.theta_3, self.theta_4, self.theta_5, self.theta_6 = smbl.symbols("theta_1 theta_2 theta_3 theta_4 theta_5 theta_6")

        # Define DH
        self.DH = [
            [self.theta_1, 0.160, 0, -smbl.pi/2],
            [self.theta_2 + smbl.pi/2, 0, -0.690, smbl.pi],
            [self.theta_3, 0, -0.690, 0],
            [self.theta_4 + smbl.pi/2, -0.195, 0, -smbl.pi/2],
            [self.theta_5, 0.195, 0, -smbl.pi/2],
            [self.theta_6, 0.085, 0, 0]
        ]

        self.c_tfm = smbl.eye(4)
        self.c_mtrx = []

        for i, p in enumerate(self.DH):
            self.c_tfm = self.c_tfm * self.compute_T_mtx(*p, print_matrix=False)
            self.c_mtrx.append(self.c_tfm)

        self.T01, self.T02, self.T03, self.T04, self.T05, self.T06 = self.c_mtrx

        # J bottom calculation
        self.J_b = smbl.Matrix.zeros(3, 6)

        self.J_b = smbl.Matrix([
            [0, self.T01[0, 2], self.T02[0, 2], self.T03[0, 2], self.T04[0, 2], self.T05[0, 2]],
            [0, self.T01[1, 2], self.T02[1, 2], self.T03[1, 2], self.T04[1, 2], self.T05[1, 2]],
            [1, self.T01[2, 2], self.T02[2, 2], self.T03[2, 2], self.T04[2, 2], self.T05[2, 2]]
        ])

        # J_top calculation.
        position_T_f_x, position_T_f_y, position_T_f_z = self.T06[0, 3], self.T06[1, 3], self.T06[2, 3]
        self.J_top = smbl.Matrix.zeros(3, 6)

        # partial differentiation of position values
        for i in range(6):
            self.J_top[0, i] = position_T_f_x.diff(smbl.symbols(f'theta_{i+1}'))
            self.J_top[1, i] = position_T_f_y.diff(smbl.symbols(f'theta_{i+1}'))
            self.J_top[2, i] = position_T_f_z.diff(smbl.symbols(f'theta_{i+1}'))

        # Jacobian matrix
        self.J = smbl.Matrix.vstack(self.J_top, self.J_b)

    # Function to compute T matrix
    def compute_T_mtx(self, theta, d, a, alpha, print_matrix=True):
        T = smbl.Matrix([
            [smbl.cos(theta), -smbl.sin(theta) * smbl.cos(alpha), smbl.sin(theta) * smbl.sin(alpha), a * smbl.cos(theta)],
            [smbl.sin(theta), smbl.cos(theta) * smbl.cos(alpha), -smbl.cos(theta) * smbl.sin(alpha), a * smbl.sin(theta)],
            [0, smbl.sin(alpha), smbl.cos(alpha), d],
            [0, 0, 0, 1]])

        if print_matrix:
            smbl.pprint(T)
        return T
    

    def move_robot(self, distance, axis, q_current_initial, publisher_obj):

        # symbol t
        t = smbl.symbols('t', real=True) 
        lnr_vlcty = 0.1       # increased to 0.1 from 0.01 to make Gazebo sim faster

        if axis == 'x_axis':
            x_pstn = lnr_vlcty * t
            y_pstn = 0
            z_pstn = 0
        
        elif axis == '-x_axis':
            x_pstn = -lnr_vlcty * t
            y_pstn = 0
            z_pstn = 0
        
        elif axis == 'y_axis':
            y_pstn = lnr_vlcty * t
            x_pstn = 0
            z_pstn = 0
        
        elif axis == '-y_axis':
            y_pstn = -lnr_vlcty * t
            x_pstn = 0
            z_pstn = 0
        
        elif axis == 'z_axis':
            z_pstn = lnr_vlcty * t
            x_pstn = 0
            y_pstn = 0
        
        elif axis == '-z_axis':
            z_pstn = -lnr_vlcty * t
            x_pstn = 0
            y_pstn = 0
        
        else:
            print ("Wrong axis chosen")
        
        # Linear vel along x, y, and z axes 
        x_dot = smbl.diff(x_pstn, t)
        y_dot = smbl.diff(y_pstn, t)
        z_dot = smbl.diff(z_pstn, t)

        # Velocity of the end effector
        end_effector_velocity = smbl.Matrix([x_dot, y_dot, z_dot, 0, 0, 0])

        # Current joint angles
        q_current = q_current_initial

        # Joint velocities
        q_dot = smbl.Matrix([])

        # Initial time as zero
        time = 0
        end_time = distance/lnr_vlcty

        # Delta t value
        delta_t = 0.05

        # Position values for plot
        x = []
        y = []
        z = []

        while time < end_time:
            print("Time: " + str(time))
            J_var = self.J.subs(
            [(self.theta_1, q_current[0]), (self.theta_2, q_current[1]), (self.theta_3, q_current[2]), (self.theta_4, q_current[3]),
             (self.theta_5, q_current[4]), (self.theta_6, q_current[5])])

            # if the determinant of the J is zero, then we break from the loop to avoid inverse issue.
            if smbl.det(J_var) == 0:
                break
            # taking inverse
            J_inv = J_var.inv()

            # Calculating Joint velocity vector using Jacobian inverse and end effector velocity vector
            q_dot = (J_inv) * end_effector_velocity.subs(t, time).evalf()
            q_current = q_current + q_dot.evalf() * delta_t
            time = time + delta_t

            position = self.T06.subs(
            [(self.theta_1, q_current[0]), (self.theta_2, q_current[1]), (self.theta_3, q_current[2]), (self.theta_4, q_current[3]),
             (self.theta_5, q_current[4]), (self.theta_6, q_current[5])])
            x.append(position[0, 3])
            y.append(position[1, 3])
            z.append(position[2, 3])


            msg = Float64MultiArray()
            q_c = nmbr.array(q_current).astype(float)
            
            # Create message to update robot joint values
            msg.data = [q_c[0,0],q_c[1,0],q_c[2,0],q_c[3,0],q_c[4,0],q_c[5,0],0.0,0.0]
            publisher_obj.publish(msg)
        
        return x, y, z


