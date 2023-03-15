package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.states.Direction;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class DriveToDoubleSubstationCommand extends DriveToPoseCommand {
    private static double doubleSubstationTargetX = FieldConstants.LoadingZone.doubleSubstationX - 1.35;

    public DriveToDoubleSubstationCommand(SJTUSwerveMK5Drivebase mDrivebase, TargetSelector mSelector) {
        super(mDrivebase, () -> {
            return new Pose2d(
                new Translation2d(doubleSubstationTargetX, mDrivebase.getLocalizer().getLatestPose().getY()),
                mSelector.getLoadingDirection() == Direction.NEAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0)
            );
        });
    }
}
