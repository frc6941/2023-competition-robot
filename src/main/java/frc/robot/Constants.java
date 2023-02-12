// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.utils.Range;
import org.frcteam6941.vision.CameraConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.motion.SuperstructureConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean TUNING = true;
    public static final boolean AUTO_TUNING = false;

    // FMS Related Information
    public static final class FMS {
        public static Alliance ALLIANCE() {
            return DriverStation.getAlliance();
        }
    }

    // Looper Configurations
    public static final double LOOPER_DT = 1.0 / 150.0; // The robot is running at 150Hz

    // CAN ID Configurations
    public static final class CANID {
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 0;
        public static final int DRIVETRAIN_FRONT_LEFT_STEER_MOTOR = 1;
        public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 2;
        public static final int DRIVETRAIN_FRONT_RIGHT_STEER_MOTOR = 3;
        public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4;
        public static final int DRIVETRAIN_BACK_LEFT_STEER_MOTOR = 5;
        public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 6;
        public static final int DRIVETRAIN_BACK_RIGHT_STEER_MOTOR = 7;

        public static final int ARM_MOTOR_LEADER = 8;
        public static final int ARM_MOTOR_FOLLOWER = 9;
        public static final int EXTENDER_MOTOR = 10;
        public static final int INTAKER_MOTOR = 11;
    }

    // Analog ID Configurations
    public static final class ANALOG_ID {
        public static final int GAMEPIECE_SENSOR = 0;
    }

    public static final class LED_CONTROL {
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 51;
    }

    // Pneumatics Configurations
    public static final class PNEUMATICS_ID {
        public static final int INTAKER_EXTENDER_FORWARD = 4;
        public static final int INTAKER_EXTENDER_REVERSE = 5;
        public static final int CLIMBER_EXTENDER_FORWARD = 6;
        public static final int CLIMBER_EXTENDER_REVERSE = 7;
    }

    // DIO Configurations
    public static final class DIO_ID {
        public static final int ARM_LIMIT_SWTICH_PORT = 0;
    }

    // Swerve Drivetrain Constants
    public static final class SUBSYSTEM_DRIVETRAIN {
        public static final double DRIVE_MAX_VELOCITY = 4.0;

        public static final double MODULE_MAX_VELOCITY = 4.0;
        public static final double MODULE_WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(4.125);

        public static final double DRIVE_GEAR_RATIO = 7.0;
        public static final double ANGLE_GEAR_RATIO = (56.0 / 6.0) * (60.0 / 10.0);
        public static final double DRIVETRAIN_SIDE_WIDTH = 0.58;
        public static final double DRIVETRAIN_SIDE_WIDTH_BUMPERED = 0.818;
        public static final Translation2d DRIVETRAIN_CENTER_OF_ROTATION = new Translation2d(0.0, 0.0);

        public static final double FRONT_LEFT_OFFSET = -74.091796875 + 90.0;
        public static final double FRONT_RIGHT_OFFSET = -128.84765625 + 90.0;
        public static final double BACK_LEFT_OFFSET = -249.43359375000003 + 90.0 + 180.0;
        public static final double BACK_RIGHT_OFFSET = -49.21875 + 90.0 + 180.0;

        public static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 50;

        public static final double DRIVETRAIN_HEADING_CONTROLLER_KP = 1.0 / 50.0;
        public static final double DRIVETRAIN_HEADING_CONTROLLER_KI = 0.00;
        public static final double DRIVETRAIN_HEADING_CONTROLLER_KD = 0.0005;
        public static final TrapezoidProfile.Constraints DRIVETRAIN_HEADING_CONTROLLER_CONSTRAINT = new TrapezoidProfile.Constraints(
                300.0, 900.0);
        public static final TrapezoidProfile.Constraints DRIVETRAIN_TARGET_POSE_FOLLOWING_CONSTRAINT = new TrapezoidProfile.Constraints(
            2.0, 1.5);

        // Note: the feedforward generated by SysID is corresponding to motor
        // voltage(which is in the range of +-12V). So when in
        // autonomous, the gain need to be resized to value ranging from -1 to 1
        // accordingly.
        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.60757, 7.6216,
                0.71241);
    }

    // Arm Constants
    public static final class SUBSYSTEM_ARM {
        public static final double MASS = 8.0;
        public static final double GEAR_RATIO = (68.0 / 8.0) * (64.0 / 18.0) * (60 / 12.0);
        public static final double HOME_ANGLE = -98.2;

        public static final double KP = 0.3;
        public static final double KI = 0.002;
        public static final double KD = 0.15;
        public static final double KF = 1023.0 / (6380.0 * 2048.0) * 0.1;
        public static final double IZONE = 500;
        public static final double CRUISE_V = 24000.0;
        public static final double CRUIVE_ACC = 15000.0;

        public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.1673, 0.10902, 0.043741, 0.0022842);
        public static final double ANGLE_OFFSET_TO_HORIZONTAL = -14.467;
    }

    // Extender Constants
    public static final class SUBSYSTEM_EXTENDER {
        public static final double GEAR_RATIO = 8.0;
        public static final double WHEEL_CIRCUMFERENCE = 24 * 0.005;
        public static final double HOME_LENGTH = 0.880;

        public static final double KP = 0.25;
        public static final double KI = 0.0003;
        public static final double KD = 0.15;
        public static final double KF = 1023.0 / (6380.0 * 2048.0) * 0.1;
        public static final double IZONE = 200;
        public static final double CRUISE_V = 25000.0;
        public static final double CRUIVE_ACC = 40000.0;
    }

    // Intake Constants
    public static final class SUBSYSTEM_INTAKE {
        public static final double INTAKING_PERCENTAGE_CONE = 0.80;
        public static final double INTAKING_PERCENTAGE_CUBE = 0.40;
        public static final double OUTTAKING_PERCENTAGE = -0.60;
        public static final double HOLD_PERCENTAGE_CUBE = 0.20;
        public static final double HOLD_PERCENTAGE_CONE = 0.50;
    }

    // Superstructure Constants
    public static final class SUBSYSTEM_SUPERSTRUCTURE {
        public static class STRUCTURE {
            public static Transform3d ROBOT_CENTER_TO_HIGH_PIVOT = new Transform3d(
                new Pose3d(), new Pose3d(0.210, 0, 1.08479, new Rotation3d()));
            public static Translation2d HIGH_PIVOT_2D_LOCATION = new Translation2d(
                ROBOT_CENTER_TO_HIGH_PIVOT.getX(),
                ROBOT_CENTER_TO_HIGH_PIVOT.getZ());
        }

        // Thresholds
        public static class THRESHOLD {
            public static double ARM = 2.0;
            public static double EXTENDER = 0.02;
        }

        // Constraints
        public static class CONSTRAINTS {
            public static Range ARM_RANGE = new Range(-98.0, 237.0);
            public static Range EXTENDER_RANGE = new Range(0.885, 1.37);
            public static Range HEIGHT_RANGE = new Range(0.05, 1.90);
            public static Range DANGEROUS_POSITIVE = new Range(230, Double.POSITIVE_INFINITY); // TODO: Need reconfirmation
            public static Range DANGEROUS_NEGATIVE = new Range(Double.NEGATIVE_INFINITY, -70.0);
            public static SuperstructureConstraint SUPERSTRUCTURE_LIMIT = new SuperstructureConstraint(
                HEIGHT_RANGE, ARM_RANGE, EXTENDER_RANGE, DANGEROUS_POSITIVE, DANGEROUS_NEGATIVE
            );
        }

        // Manual Deltas
        public static class MANUAL_DELTA {
            public static double ANGLE_CHANGE_DELTA = 0.5;
            public static double LENGTH_CHANGE_DELTA = 0.01;
        }
    }

    // Vision Constants
    public static final class SUBSYSTEM_VISION {
        public static final CameraConstants[] CAMERA_CONSTANTS = new CameraConstants[] {
            new CameraConstants(
                "IP_VM1",
                new Pose3d(0.0, 0.112, 0.775, 
                new Rotation3d(0.0, 0.0, -Math.PI))
            ),
            new CameraConstants(
                "IP_VM2",
                new Pose3d(0.0, -0.135, 0.863,
                new Rotation3d(0.0, 0.0, 0.0))
            )
        };
    }

    // Controller
    public static final class CONTROLBOARD {
        public static final double CONTROLLER_DEADBAND = 0.05;

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final boolean CONTROLLER_INVERT_X = false;
        public static final boolean CONTROLLER_INVERT_Y = false;
        public static final boolean CONTROLLER_INVERT_R = false;
        public static final double CONTROLLER_PEDAL = 0.8;

        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
}
