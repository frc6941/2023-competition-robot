package frc.robot.commands;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveToSingleSubstationCommand extends DriveToPoseCommand {
    private static final LoggedTunableNumber singleSubstationX = new LoggedTunableNumber("Single Substation X", FieldConstants.LoadingZone.singleSubstationCenterX);
    private static final LoggedTunableNumber singleSubstationY = new LoggedTunableNumber("Single Substation Y", FieldConstants.LoadingZone.leftY - 0.5);

    public DriveToSingleSubstationCommand(SJTUSwerveMK5Drivebase mDriveBase) {
        super(mDriveBase, () -> 
            new Pose2d(
                new Translation2d(singleSubstationX.get(), singleSubstationY.get()),
                Rotation2d.fromDegrees(-90.0)
        ));
    }
}
