package frc.robot.commands;

import java.util.function.Supplier;

import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.motion.PathProvider;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveAlongPath extends CommandBase {
    private SJTUSwerveMK5Drivebase mDrivebase;
    private PathProvider mPathProvider;
    private Supplier<Pose2d> targetPose;
    private Supplier<Obstacle[]> obstacles;

    private Timer pathTrackingTimer = new Timer();

    private ProfiledPIDController poseAssistXController = new ProfiledPIDController(1.5, 0.001, 0.0,
            new Constraints(3.5, 3.0));
    private ProfiledPIDController poseAssistYController = new ProfiledPIDController(1.5, 0.001, 0.0,
            new Constraints(3.5, 3.0));

    private Runnable r = new Runnable() {
        @Override
        public void run() {
            mPathProvider.getPath().ifPresent(path -> {
                pathTrackingTimer.start();
                Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
                Pose2d trackingPose = new Pose2d(path.getPathPointByDistance(pathTrackingTimer.get() * 2.5),
                        targetPose.get().getRotation());
                double xOut = poseAssistXController.calculate(currentPose.getX(), trackingPose.getX());
                double yOut = poseAssistYController.calculate(currentPose.getY(), trackingPose.getY());

                mDrivebase.setLockHeading(true);
                mDrivebase.setHeadingTarget(trackingPose.getRotation().getDegrees());
                mDrivebase.drive(new Translation2d(xOut, yOut), 0.0, true, false);
            });
        }
    };

    private Notifier n = new Notifier(r);

    public DriveAlongPath(SJTUSwerveMK5Drivebase mDrivebase,
            PathProvider mPathProvider, Supplier<Pose2d> targetPose, Supplier<Obstacle[]> obstacles) {
        this.mDrivebase = mDrivebase;
        this.mPathProvider = mPathProvider;
        this.targetPose = targetPose;
        this.obstacles = obstacles;

        poseAssistXController.setIntegratorRange(-0.2, 0.2);
        poseAssistYController.setIntegratorRange(-0.2, 0.2);
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {
        pathTrackingTimer.reset();
        pathTrackingTimer.stop();
        mPathProvider.clear();

        Pose2d currentPosition = mDrivebase.getLocalizer().getLatestPose();
        Pose2d currentVelocity = mDrivebase.getLocalizer().getMeasuredVelocity();
        poseAssistXController.reset(currentPosition.getX(), currentVelocity.getX());
        poseAssistYController.reset(currentPosition.getY(), currentVelocity.getY());
        mDrivebase.resetHeadingController();
        mDrivebase.setLockHeading(false);

        mPathProvider.buildPath(
                mDrivebase.getLocalizer().getLatestPose().getTranslation(),
                targetPose.get().getTranslation(),
                obstacles.get()
        );
        
        n.startPeriodic(Constants.LOOPER_DT);  
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
        mDrivebase.setLockHeading(false);
        n.stop();
        pathTrackingTimer.reset();
        pathTrackingTimer.stop();
        mPathProvider.clear();
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Transform2d delta = targetPose.get().minus(currentPose);
        return delta.getTranslation().getNorm() < 0.05 && Math.abs(delta.getRotation().getDegrees()) < 3.0;
    }
}
