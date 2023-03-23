package frc.robot.commands;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.states.Direction;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class DriveToDoubleSubstationCommand extends DriveToPoseCommand {
    private static LoggedTunableNumber doubleSubstationTargetX = new LoggedTunableNumber("Double Substation X", 5.65);

    public DriveToDoubleSubstationCommand(SJTUSwerveMK5Drivebase mDrivebase, TargetSelector mSelector) {
        super(mDrivebase, () -> {
            if(doubleSubstationTargetX.hasChanged()) {
                System.out.println("Double Substation X Changed!");
            }

            return new Pose2d(
                new Translation2d(doubleSubstationTargetX.get(), mDrivebase.getLocalizer().getLatestPose().getY()),
                mSelector.getLoadingDirection() == Direction.NEAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0)
            );
        });
    }
}
