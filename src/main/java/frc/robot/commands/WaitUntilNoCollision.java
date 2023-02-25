package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.utils.AllianceFlipUtil;

public class WaitUntilNoCollision extends WaitUntilCommand{
    public static final double minDriveX = FieldConstants.Grids.outerX + 1.0;
    public WaitUntilNoCollision(Supplier<Pose2d> pose) {
        super(() -> AllianceFlipUtil.apply(pose.get()).getX() > minDriveX);
    }
}
