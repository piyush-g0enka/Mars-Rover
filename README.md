# Mars Rover

## Introduction

This project focuses on implementing forward and inverse kinematics on a teleoperable mobile rover with a manipulator arm attached. The implementation is validated using Sympy and Gazebo simulation.

![Gif](https://github.com/piyush-g0enka/Mars-Rover/blob/main/mars-rover-gif.gif)
## Application

The robot is a tele-operable rover with a multi-purpose arm, equipped with a toolbox of end-effectors. For this project, a vacuum gripper is utilized as the end effector to demonstrate forward and inverse kinematics. Other end-effector tools could be implemented in the future based on the robot's application requirements.



## Robot Type

The system consists of a mobile rover with a manipulator arm.

## DOFs and Dimensions

- Rover: Three degrees of freedom
  - Height: 850mm
  - Length: 1694mm
  - Breadth: 750mm

- Manipulator Arm: Six degrees of freedom
  - Height: 1.73 meters
  - Joints: All joints are of revolute type

## CAD Model
![cad](https://github.com/piyush-g0enka/Mars-Rover/blob/main/cad-render.png)
The CAD model includes a rover with a manipulator arm. The rover is rear wheel-driven, and the front wheels are individually steerable. A tool set holder at the rear can hold multiple end-effectors. The demonstration of end-effector changes is considered as future work.

## Forward Kinematics

Forward kinematics involves calculating the position and orientation of each link, including the end-effector, based on joint variables. The final transformation matrix between the final link and the base link is used for this purpose.

## Inverse Kinematics

Inverse Kinematics (IK) involves finding suitable joint configurations for smooth and accurate movement of end effectors. The joint configurations are represented by joint angles. The Jacobian matrix is used for this application.

## Gazebo and Rviz Visualization

The robot is tele-operated using keys, controlling the robot, end-effector, and interacting with the environment (table and cube box). The goal is for the user to pick up the cube and place it at their desired location. Code execution instructions are available in the README file.

## Problems Faced

Challenges included joint misalignment in URDF compared to SolidWorks model, which was resolved by adjusting joint axis in URDF. Another challenge was joints breaking in Gazebo due to lacking velocity controllers in URDF, resolved by adding velocity controllers.

## Lessons Learned

Practical implementation of forward and inverse kinematics, end-to-end development of a robot from 3D modeling to simulation, and the importance of joint configuration consistency were learned in this project.

## Conclusion

The project successfully implements forward and inverse kinematics for a tele-operable mobile rover with a manipulator arm. The validation through Sympy and Gazebo simulation demonstrates precise control over the end-effector, overcoming challenges and ensuring project resilience.

## Future Work

1. Adding path planning/motion planning and perception capabilities.
2. Designing and adding other end effectors, demonstrating the manipulator's end-effector changing capacity.
