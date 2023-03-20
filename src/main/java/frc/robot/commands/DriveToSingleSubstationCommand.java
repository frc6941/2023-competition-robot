package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveToSingleSubstationCommand extends DriveToPoseCommand {
    private static final double singleSubstationX = FieldConstants.LoadingZone.singleSubstationCenterX;
    private static final double singleSubstationTargetY = FieldConstants.LoadingZone.leftY + 0.4;

    public DriveToSingleSubstationCommand(SJTUSwerveMK5Drivebase mDriveBase) {
        super(mDriveBase, () -> 
            new Pose2d(
                new Translation2d(singleSubstationX, singleSubstationTargetY),
                Rotation2d.fromDegrees(180.0)
        ));
    }
}
