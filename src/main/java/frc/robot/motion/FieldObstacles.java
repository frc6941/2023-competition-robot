package frc.robot.motion;

import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.astar.obstacles.RectangularObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle.AXIS;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle.DIRECTION;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class FieldObstacles {
    private static double margin = Constants.SUBSYSTEM_DRIVETRAIN.DRIVETRAIN_SIDE_WIDTH_BUMPERED * 0.5 + 0.40;
    private static Translation2d smallBoundary = FieldConstants.allianceFlip(new Translation2d(
            FieldConstants.Grids.outerX,
            FieldConstants.Community.rightY));
    private static Translation2d largeBoundary = FieldConstants.allianceFlip(new Translation2d(
            FieldConstants.fieldLength * 0.5,
            FieldConstants.Community.leftY));

    public static Obstacle boundaryXSmall = new InfiniteBarrierObstacle(
            smallBoundary.getX(), DIRECTION.SMALLER, AXIS.X);
    public static Obstacle boundaryXLarge = new InfiniteBarrierObstacle(
            largeBoundary.getX(), DIRECTION.GREATER, AXIS.X);
    public static Obstacle boundaryYSmall = new InfiniteBarrierObstacle(
            smallBoundary.getY(), DIRECTION.SMALLER, AXIS.Y);
    public static Obstacle boundaryYLarge = new InfiniteBarrierObstacle(
            largeBoundary.getY(), DIRECTION.GREATER, AXIS.Y);
    public static Obstacle boundaryChargingStation = new RectangularObstacle(
            FieldConstants.Community.chargingStationCorners,
            margin,
            margin);
    
    public static Obstacle[] obstacleList = new Obstacle[] { boundaryXSmall, boundaryXLarge, boundaryYSmall, boundaryYLarge, boundaryChargingStation};
}
