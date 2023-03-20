# 2023CompeitionRobot

## Project Structure

- auto: Autonomous routines.
- controlboard: Read inputs and outputs from the driver and the operator.
- motion: Kinematics, inverse kinematics, and constraints of superstructure.
- shuffleboard: Displaying and logging.
- states: The states of Superstructure, Scoring, and the whole Robot.
- subsystems: All the small functional components on the robot.
- utils: convenience features.

## Positive Direcitions

1. **Counter-Clockwise (CCW)** is taken as the positive direction - unless explicitly stated.
2. Pointing the right thumb to the positive direction, the direction of the rest of the fingers when they are curled up naturally is the positive direction of the angle. (Right hand rule)
3. The direction of the inclined tower is the positive direciton. The arrows shown on the picture indicates the positive direction of drivetrain and arm.

![Positive Directions](positive-directions.png)

## Alliance Flip

This season requires flip of poses according to alliance, as the field is centric symmetric.
As a convention, the origin is set at the **lower right corner** of the blue alliance. With forward being x positive, leftward being y positive.

All the poses calculated are based on the blue alliance, and flips are arried out **at the base commands**, like DriveToPose command and FollowTrajectory command.
Yet, the robot pose calculated is unflipped. So, when calculating, the pose need to be flipped back.

In short summary, the AllianceFlipUtil is applied in only two conditions:

1. Flip the pose from the robot
2. Carry out actual action at base commands

At other places, the flip should not be carried out.

