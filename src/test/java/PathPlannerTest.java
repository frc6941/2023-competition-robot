import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import java.time.Duration;
import java.util.ArrayList;

import org.frcteam6941.pathplanning.astar.AStarPathPlanner;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.astar.obstacles.RectangularObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle.AXIS;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle.DIRECTION;
import org.frcteam6941.pathplanning.trajectory.TrajectoryGenerationUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;

public class PathPlannerTest {
    @BeforeEach
    void setup() {

    }

    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void overtimeTest() {
        AStarPathPlanner pathPlanner = new AStarPathPlanner();
        Obstacle[] obstacles = new Obstacle[] {
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.X),
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.Y),
                new InfiniteBarrierObstacle(16.0, DIRECTION.GREATER, AXIS.X),
                new InfiniteBarrierObstacle(8.0, DIRECTION.GREATER, AXIS.Y)
        };
        Translation2d startPoint = new Translation2d();
        Translation2d endPoint = new Translation2d(7.0, 7.0);

        assertEquals(null, pathPlanner.plan(startPoint, endPoint, obstacles, 0.1, 0.1, 0.15, 1.0));
    }

    @Test
    void efficiencyTest() {
        AStarPathPlanner pathPlanner = new AStarPathPlanner();
        Obstacle[] obstacles = new Obstacle[] {
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.X),
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.Y),
                new InfiniteBarrierObstacle(16.0, DIRECTION.GREATER, AXIS.X),
                new InfiniteBarrierObstacle(8.0, DIRECTION.GREATER, AXIS.Y),
                new RectangularObstacle(
                        FieldConstants.Community.chargingStationCorners)
        };
        Translation2d proposedPoint = new Translation2d(7.74, 2.91);
        Translation2d targetPoint = new Translation2d(2.04, 2.75);

        assertNotEquals(null, pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, 0.2));
        assertNotEquals(null, pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, 0.2));
        assertNotEquals(null, pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, 0.2));
        assertNotEquals(null, pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, 0.2));
        assertNotEquals(null, pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, 0.2));
    }

    @Test
    void resultSameTest() {
        AStarPathPlanner pathPlanner = new AStarPathPlanner();
        Obstacle[] obstacles = new Obstacle[] {
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.X),
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.Y),
                new InfiniteBarrierObstacle(16.0, DIRECTION.GREATER, AXIS.X),
                new InfiniteBarrierObstacle(8.0, DIRECTION.GREATER, AXIS.Y),
                new RectangularObstacle(
                        FieldConstants.Community.chargingStationCorners)
        };
        Translation2d proposedPoint = new Translation2d(7.74, 2.91);
        Translation2d targetPoint = new Translation2d(2.04, 2.75);
        assertEquals(
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY).size(),
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY)
                        .size());
        assertEquals(
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY).size(),
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY)
                        .size());
        assertEquals(
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY).size(),
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.001, Double.POSITIVE_INFINITY)
                        .size());
        assertEquals(
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY).size(),
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.001, Double.POSITIVE_INFINITY)
                        .size());
        assertEquals(
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.0, Double.POSITIVE_INFINITY).size(),
                pathPlanner.plan(proposedPoint, targetPoint, obstacles, 0.1, 0.1, 1.001, Double.POSITIVE_INFINITY)
                        .size());
    }

    @Test
    void trajectoryGenerationTest() {
        AStarPathPlanner pathPlanner = new AStarPathPlanner();
        Translation2d startPoint = new Translation2d(7.74, 2.91);
        Translation2d endPoint = new Translation2d(2.04, 2.75);
        Obstacle[] obstacles = new Obstacle[] {
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.X),
                new InfiniteBarrierObstacle(0, DIRECTION.SMALLER, AXIS.Y),
                new InfiniteBarrierObstacle(16.0, DIRECTION.GREATER, AXIS.X),
                new InfiniteBarrierObstacle(8.0, DIRECTION.GREATER, AXIS.Y),
                new RectangularObstacle(
                        FieldConstants.Community.chargingStationCorners)
        };

        ArrayList<Translation2d> result = pathPlanner.plan(startPoint, endPoint, obstacles, 0.1, 0.1, 1.0, 1.0);
        PathConstraints constraint = new PathConstraints(3.5, 5.0);

        assertTimeoutPreemptively(Duration.ofSeconds((long) 1.0), () -> TrajectoryGenerationUtils.generatePathPlannerTrajectory(result, constraint));
    }
}
