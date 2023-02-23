package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class AutoSkewScoreCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    TargetSelector mTargetSelector;
    ArmAndExtender mSuperstructure;

    Supplier<Pose2d> drivetrainTarget;
    Supplier<Translation2d> endEffectorPosition2dTarget;

    public AutoSkewScoreCommand(SJTUSwerveMK5Drivebase mDrivebase, TargetSelector mTargetSelector, ArmAndExtender mSuperstructure) {
        this.mDrivebase = mDrivebase;
        this.mTargetSelector = mTargetSelector;
        this.mSuperstructure = mSuperstructure;
    }
    

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // public Supplier<Pose2d> buildPoseSupplier() {
    //     Pose2d currentPose = mDrivebase.getLocalizer().getLatestPose();

    // }

    // public Supplier<Translation2d> buildEndEffectorPositonSupplier() {

    // }
}
