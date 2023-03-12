package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.utils.AllianceFlipUtil;

public class WaitUntilNoCollision extends SequentialCommandGroup{
    public static final double minDriveX = FieldConstants.Grids.outerX + 0.7;
    public static final double maxDriveX = FieldConstants.LoadingZone.doubleSubstationX - 2.5;
    public WaitUntilNoCollision(Supplier<Pose2d> pose) {
        addCommands(
            new WaitUntilCommand(() -> AllianceFlipUtil.apply(pose.get()).getX() > minDriveX && AllianceFlipUtil.apply(pose.get()).getX() < maxDriveX)
        );
    }
}
