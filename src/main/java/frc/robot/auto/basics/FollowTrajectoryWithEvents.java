package frc.robot.auto.basics;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class FollowTrajectoryWithEvents extends FollowPathWithEvents {
    public FollowTrajectoryWithEvents(SJTUSwerveMK5Drivebase mDrivebase, PathPlannerTrajectory trajectory,
            HashMap<String, Command> eventMap, boolean angleLock, boolean reset, boolean requiredOnTarget) {
        super(
                new FollowTrajectory(mDrivebase, trajectory, angleLock, reset, requiredOnTarget),
                trajectory.getMarkers(),
                eventMap
            );
    }
}
