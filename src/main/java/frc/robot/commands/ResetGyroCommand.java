package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.utils.AllianceFlipUtil;

public class ResetGyroCommand extends InstantCommand {
    public ResetGyroCommand(SJTUSwerveMK5Drivebase mDrivebase, Rotation2d angle) {
        super(() -> {
            Rotation2d finalAngle;
            if(DriverStation.getAlliance() == Alliance.Red) {
                finalAngle = angle.plus(Rotation2d.fromDegrees(180.0));
            } else {
                finalAngle = angle;
            }
            mDrivebase.resetYaw(finalAngle.getDegrees());
        });
    }
}
