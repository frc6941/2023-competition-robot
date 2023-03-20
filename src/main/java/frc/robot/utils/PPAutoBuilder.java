package frc.robot.utils;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.auto.basics.FollowTrajectoryWithEvents;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class PPAutoBuilder extends BaseAutoBuilder{
    SJTUSwerveMK5Drivebase mDrivebase;

    public PPAutoBuilder(SJTUSwerveMK5Drivebase mDrivebase, HashMap<String, Command> eventMap) {
        super(
            () -> mDrivebase.getLocalizer().getLatestPose(),
            (resetPose) -> mDrivebase.resetPose(AllianceFlipUtil.apply(resetPose)),
            eventMap,
            DrivetrainType.HOLONOMIC,
            false // Invert should be handle by user
        );
        this.mDrivebase = mDrivebase;
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new FollowTrajectory(this.mDrivebase, trajectory);
    }

    @Override
    public CommandBase followPathWithEvents(PathPlannerTrajectory trajectory) {
        return new FollowTrajectoryWithEvents(this.mDrivebase, trajectory, (HashMap<String, Command>) eventMap);
    }
}