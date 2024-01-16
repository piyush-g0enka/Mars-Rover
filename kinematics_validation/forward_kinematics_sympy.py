#!/usr/bin/env python
# coding: utf-8

# In[1]:


# importing libraries
import sympy as sp
import math

#defining a class for forward kinematics
class RobotArm:
    def __init__(self):
        self.theta_1, self.theta_2, self.theta_3, self.theta_4, self.theta_5, self.theta_6 = sp.symbols("theta_1 theta_2 theta_3 theta_4 theta_5 theta_6")
        
        self.DH_parameters = [
            [self.theta_1, 0.160, 0, -sp.pi/2],
            [self.theta_2 + sp.pi/2, 0, -0.690, sp.pi],
            [self.theta_3, 0, -0.690, 0],
            [self.theta_4 + sp.pi/2, -0.195, 0, -sp.pi/2],
            [self.theta_5, 0.195, 0, -sp.pi/2],
            [self.theta_6, 0.085, 0, 0]
        ]
        
        # declaration of cumulative transformation matrix
        # identity matrix is declared here
        self.cumulative_transform = sp.eye(4)
        self.cumulative_matrices = []
        self.compute_transform_matrices()

    # DH transformation matrix equation
    def transformation_matrix(self, theta, d, a, alpha):
        T = sp.Matrix([
            [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
            [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), d],
            [0, 0, 0, 1]])

        return T

    #method to compute transformation matrix. These matrices are added to the cumulative matrix.
    def compute_transform_matrices(self):
        for i, params in enumerate(self.DH_parameters):
            self.cumulative_transform = self.cumulative_transform * self.transformation_matrix(*params)
            self.cumulative_matrices.append(self.cumulative_transform)
    
    #method for final transformation matrix 
    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):
        T_f = self.cumulative_matrices[-1].subs([(self.theta_1, theta1), (self.theta_2, theta2),
                                                  (self.theta_3, theta3), (self.theta_4, theta4),
                                                  (self.theta_5, theta5), (self.theta_6, theta6)])
        return T_f

#  Forward Kinematics
robot = RobotArm()
# all joint angles as zero
end_effector_position = robot.forward_kinematics(0,0,0,0,0,0)
# getting rounded end effector position
rounded_end_effector_position = end_effector_position.evalf().applyfunc(lambda x: round(x, 3))
#printing
sp.pprint(rounded_end_effector_position, use_unicode=True, num_columns=120, wrap_line=False)

