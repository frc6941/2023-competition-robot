package frc.robot.commands;

import java.util.function.Supplier;

import org.frcteam6941.pathplanning.astar.AStarPathPlanner;
import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.smoothing.FloydTrimming;
import org.frcteam6941.pathplanning.universal.Path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;

public class RequestSuperstructureFollowTrajectory extends CommandBase {
    ArmAndExtender mSuperstructure;
    Supplier<SuperstructureState> start;
    Supplier<SuperstructureState> end;
    Obstacle[] trajectoryObstacles;
    double stepAngle;
    double stepLength;

    Path superstructurePath;

    private double speed = 50.0;
    private Timer timer = new Timer();


    public RequestSuperstructureFollowTrajectory(ArmAndExtender mSuperstructure, Supplier<SuperstructureState> start, Supplier<SuperstructureState> end, Obstacle[] trajectoryObstacles, double stepAngle, double stepLength) {
        this.mSuperstructure = mSuperstructure;
        this.start = start;
        this.end = end;
        this.trajectoryObstacles = trajectoryObstacles;
        this.stepAngle = stepAngle;
        this.stepLength = stepLength;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {
        AStarPathPlanner planner = new AStarPathPlanner();
        this.superstructurePath = FloydTrimming.trimPath(
            planner.plan(start.get().toPolar(), end.get().toPolar(), trajectoryObstacles, stepAngle, stepLength, 4.0, 10.0)
        );
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(superstructurePath == null) {
            return;
        }
        Translation2d targetState = superstructurePath.getPathPointByDistance(timer.get() * speed);
        SuperstructureState superstructureTarget = new SuperstructureState(Rotation2d.fromDegrees(targetState.getX()), targetState.getY());
        mSuperstructure.setSuperstructureState(superstructureTarget);
    }

    @Override
    public void end(boolean interrupted) {
        timer.reset();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.getCurrentSuperstructureState().isOnTarget(end.get(), 1.0, 0.02);
    }
}
