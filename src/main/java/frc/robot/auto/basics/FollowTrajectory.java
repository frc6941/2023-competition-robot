package frc.robot.auto.basics;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class FollowTrajectory extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    Trajectory targetTrajectory;
    
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final SimpleMotorFeedforward feedforward;

    private static final double maxVelocity = 3.0;
    private static final double maxAcceleration = 3.0;
    private static final double maxCentripetalAcceleration = 3.0;

    public FollowTrajectory(
        SJTUSwerveMK5Drivebase mDrivebase,
        Pose2d startPose, Pose2d endPose, List<Translation2d> innerPoints,
        List<TrajectoryConstraint> constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
                .setKinematics(mDrivebase.getKinematics())
                .setEndVelocity(0.0)
                .setStartVelocity(0.0)
                .addConstraints(constraints)
                .addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAcceleration));

        targetTrajectory = TrajectoryGenerator.generateTrajectory(startPose, innerPoints, endPose, config);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
    }
}
