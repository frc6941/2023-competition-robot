package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveToPoseCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    // Pose Assist Controller
    private ProfiledPIDController poseAssistXController = new ProfiledPIDController(0.9, 0.001, 0.0,
            new Constraints(3.5, 3.5));
    private ProfiledPIDController poseAssistYController = new ProfiledPIDController(0.9, 0.001, 0.0,
            new Constraints(3.5, 3.5));

    private Supplier<Pose2d> targetPose;

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Pose2d> targetPose) {
        this.mDrivebase = mDrivebase;
        this.targetPose = targetPose;
        poseAssistXController.setIntegratorRange(-0.2, 0.2);
        poseAssistYController.setIntegratorRange(-0.2, 0.2);
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = mDrivebase.getLocalizer().getLatestPose();
        Pose2d currentVelocity = mDrivebase.getLocalizer().getMeasuredVelocity();
        poseAssistXController.reset(currentPosition.getX(), currentVelocity.getX());
        poseAssistYController.reset(currentPosition.getY(), currentVelocity.getY());
        mDrivebase.resetHeadingController();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        double xOut = poseAssistXController.calculate(currentPose.getX(), targetPose.get().getX());
        double yOut = poseAssistYController.calculate(currentPose.getY(), targetPose.get().getY());
        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(targetPose.get().getRotation().getDegrees());
        mDrivebase.drive(new Translation2d(xOut, yOut), 0.0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Transform2d delta = targetPose.get().minus(currentPose);
        return delta.getTranslation().getNorm() < 0.10 && Math.abs(delta.getRotation().getDegrees()) < 5.0;
    }
}
