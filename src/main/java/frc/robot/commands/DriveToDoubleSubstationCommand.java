package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.states.Direction;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;

public class DriveToDoubleSubstationCommand extends SequentialCommandGroup {
    private double doubleSubstationTargetX = FieldConstants.LoadingZone.doubleSubstationX - 1.35;
    private double doubleSubstationLeftY = FieldConstants.LoadingZone.doubleSubstationCenterY + 0.8;
    private double doubleSubstationRightY = FieldConstants.LoadingZone.doubleSubstationCenterY - 0.8;
    private List<Translation2d> doubleSubstationTranslations = List.of(
        new Translation2d(doubleSubstationTargetX, doubleSubstationLeftY),
        new Translation2d(doubleSubstationTargetX, doubleSubstationRightY)
    );

    private Command dash;

    public DriveToDoubleSubstationCommand(SJTUSwerveMK5Drivebase mDrivebase, TargetSelector mSelector) {
        dash = new DriveToPoseCommand(mDrivebase, () -> {
            Pose2d currentPose = AllianceFlipUtil.apply(mDrivebase.getLocalizer().getLatestPose());
            Translation2d target = new Translation2d(doubleSubstationTargetX, doubleSubstationRightY);
            return new Pose2d(
                target,
                mSelector.getLoadingDirection() == Direction.NEAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0)
            );
        });

        addCommands(
            dash
        );
    }
}
