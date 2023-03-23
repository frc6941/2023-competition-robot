package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.utils.AllianceFlipUtil;

public class DriveSnapRotationCommand extends FunctionalCommand {
    SJTUSwerveMK5Drivebase mDrivebase;
    Supplier<Rotation2d> snapRotationTarget;
    public DriveSnapRotationCommand(SJTUSwerveMK5Drivebase mDrivebase, Supplier<Rotation2d> snapRotationTarget) {
        super(
            () -> {
                mDrivebase.setLockHeading(true);
            },
            () -> {
                mDrivebase.setLockHeading(true);
                mDrivebase.setHeadingTarget(AllianceFlipUtil.apply(snapRotationTarget.get()).getDegrees());
            },
            (interrupted) -> {
            },
            () -> true,
            new Subsystem[] {}
        );
    }
}