package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.utils.AllianceFlipUtil;

public class DriveToPoseCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    // Pose Assist Controller
    private ProfiledPIDController driveController = new ProfiledPIDController(2.5, 0.005, 0, Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_TRANSLATIONAL_CONSTRAINT);

    private Supplier<Pose2d> targetPose;

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Pose2d> targetPose) {
        this.mDrivebase = mDrivebase;
        this.targetPose = targetPose;
        driveController.setIntegratorRange(-0.5, 0.5);
        addRequirements(mDrivebase);
    }

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Pose2d targetPose) {
        this(mDrivebase, () -> targetPose);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Translation2d deltaTranslation = targetPose.get().getTranslation().minus(currentPose.getTranslation());
        driveController.reset(
            deltaTranslation.getNorm()
        );
        mDrivebase.resetHeadingController();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Pose2d transformedPose = AllianceFlipUtil.apply(targetPose.get());

        Translation2d deltaTranslation = currentPose.getTranslation().minus(transformedPose.getTranslation());
        double driveGain = driveController.calculate(deltaTranslation.getNorm(), 0.0);
        Translation2d velocity = new Translation2d(driveGain, deltaTranslation.getAngle().plus(
            AllianceFlipUtil.shouldFlip() ? Rotation2d.fromDegrees(180.0) : new Rotation2d()
        ));
        System.out.println(currentPose.getTranslation());

        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(transformedPose.getRotation().getDegrees());
        if(deltaTranslation.getNorm() < 0.03) {
            mDrivebase.stopMovement();
        } else {
            mDrivebase.drive(velocity, 0.0, true, false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Transform2d delta = AllianceFlipUtil.apply(targetPose.get()).minus(currentPose);
        return delta.getTranslation().getNorm() < 0.05 && Math.abs(delta.getRotation().getDegrees()) < 1.0;
    }
}
