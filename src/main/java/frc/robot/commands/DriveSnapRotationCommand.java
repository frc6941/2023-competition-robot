package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveSnapRotationCommand extends FunctionalCommand {
    SJTUSwerveMK5Drivebase mDrivebase;
    Supplier<Rotation2d> snapRotationTarget;
    public DriveSnapRotationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Rotation2d> snapRotationTarget) {
        super(
            mDrivebase::resetHeadingController,
            () -> {
                mDrivebase.setLockHeading(true);
                mDrivebase.setHeadingTarget(snapRotationTarget.get().getDegrees());
            },
            (interrupted) -> {
                mDrivebase.setLockHeading(false);
            },
            () -> false,
            new Subsystem[] {}
        );
    }
}