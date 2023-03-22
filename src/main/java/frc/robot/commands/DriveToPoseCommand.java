package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.utils.AllianceFlipUtil;

public class DriveToPoseCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    // Pose Assist Controller
    private ProfiledPIDController driveControllerLimited = new ProfiledPIDController(
        4.0, 0.00, 0,
        Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_TRANSLATIONAL_CONSTRAINT
    );
    private ProfiledPIDController driveControllerUnlimited = new ProfiledPIDController(
        4.0, 0.00, 0,
        Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_TRANSLATIONAL_CONSTRAINT
    );
    private boolean limited = true;

    private Supplier<Pose2d> targetPose;

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Pose2d> targetPose) {
        this.mDrivebase = mDrivebase;
        this.targetPose = targetPose;
        driveControllerLimited.setIntegratorRange(-0.5, 0.5);
        addRequirements(mDrivebase);
    }

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Pose2d> targetPose, boolean limited) {
        this.mDrivebase = mDrivebase;
        this.targetPose = targetPose;
        driveControllerLimited.setIntegratorRange(-0.5, 0.5);
        this.limited = limited;
        addRequirements(mDrivebase);
    }

    public DriveToPoseCommand(SJTUSwerveMK5Drivebase mDrivebase, Pose2d targetPose) {
        this(mDrivebase, () -> targetPose);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Pose2d currentVelocity = mDrivebase.getLocalizer().getMeasuredVelocity();
        Pose2d transformedPose = AllianceFlipUtil.apply(targetPose.get());
        Translation2d deltaTranslation = transformedPose.getTranslation().minus(currentPose.getTranslation());
        double dot = currentVelocity.getX() * deltaTranslation.getX() + currentVelocity.getY() * deltaTranslation.getY();
        driveControllerLimited.reset(
            deltaTranslation.getNorm(),
            dot / deltaTranslation.getNorm()
        );
        mDrivebase.resetHeadingController();
        mDrivebase.setLockHeading(false);
    }

    @Override
    public void execute() {
        Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();
        Pose2d transformedPose = AllianceFlipUtil.apply(targetPose.get());

        Translation2d deltaTranslation = currentPose.getTranslation().minus(transformedPose.getTranslation());
        
        double driveGain;
        if(limited) {
            driveGain = driveControllerLimited.calculate(deltaTranslation.getNorm(), 0.0);
        } else {
            driveGain = driveControllerUnlimited.calculate(deltaTranslation.getNorm(), 0.0);
        }
        
        Translation2d velocity = new Translation2d(driveGain, deltaTranslation.getAngle());
        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(transformedPose.getRotation().getDegrees());
        mDrivebase.drive(velocity, 0.0, true, false, false);
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
