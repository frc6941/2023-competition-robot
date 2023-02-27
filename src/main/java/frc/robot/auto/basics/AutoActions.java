package frc.robot.auto.basics;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class AutoActions {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSuperstructure;
    TargetSelector mTargetSelector;

    private static final TrajectoryConstraint driveVelocityAndAccelerationConstraint = new TrajectoryCon(3.0, 2.0);

    public AutoActions() {
    }

    public static Command path(Pose2d startPose, Pose2d endPose) {

        
        return new SequentialCommandGroup(

        );
    }
}
