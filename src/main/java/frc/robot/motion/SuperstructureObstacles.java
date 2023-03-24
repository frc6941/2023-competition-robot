package frc.robot.motion;

import org.frcteam6941.pathplanning.astar.obstacles.CircleObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.astar.obstacles.RectangularObstacle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.states.SuperstructureState;

public class SuperstructureObstacles {
    public static Obstacle minRadiusObstacle = new CircleObstacle(new Translation2d(), Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min);
    public static Obstacle maxRadiusObstacle = new CircleObstacle(new Translation2d(), Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.max);
    public static Obstacle drivetrainObstacle = new RectangularObstacle(
        new SuperstructureState(Rotation2d.fromDegrees(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_POSITIVE.min), Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min).toPolar(),
        new SuperstructureState(Rotation2d.fromDegrees(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_POSITIVE.min), Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.max).toPolar(),
        new SuperstructureState(Rotation2d.fromDegrees(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_NEGATIVE.max), Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min).toPolar(),
        new SuperstructureState(Rotation2d.fromDegrees(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_NEGATIVE.max), Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.max).toPolar()
    );

    public static Obstacle[] get() {
        return new Obstacle[] { 
            minRadiusObstacle, maxRadiusObstacle
        };
    }
}
