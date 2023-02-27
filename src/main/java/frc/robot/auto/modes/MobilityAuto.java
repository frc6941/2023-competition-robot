package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class MobilityAuto extends AutoModeBase {
    public String autoName = "Mobility Auto";

    public Pose2d startingPose;

    public MobilityAuto() {
        autoCommand = new DriveTeleopCommand(SJTUSwerveMK5Drivebase.getInstance(), ArmAndExtender.getInstance(), () -> new Translation2d(0.5, 0.0), () -> 0.0, false).repeatedly();
    }
    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }    
}
