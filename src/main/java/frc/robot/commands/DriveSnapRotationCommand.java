package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.utils.AllianceFlipUtil;

public class DriveSnapRotationCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    Supplier<Rotation2d> snapRotationTarget;
    double epsilon = 0.5;

    public DriveSnapRotationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Rotation2d> snapRotationTarget, double epsilon) {
        this.mDrivebase = mDrivebase;
        this.snapRotationTarget = snapRotationTarget;
        this.epsilon = epsilon;
    }

    public DriveSnapRotationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Rotation2d> snapRotationTarget) {
        this.mDrivebase = mDrivebase;
        this.snapRotationTarget = snapRotationTarget;
    }

    @Override
    public void initialize() {
        mDrivebase.setLockHeading(true);
        mDrivebase.resetHeadingController();
    }

    @Override
    public void execute() {
        mDrivebase.setLockHeading(true);
        mDrivebase.setHeadingTarget(AllianceFlipUtil.apply(snapRotationTarget.get()).getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
