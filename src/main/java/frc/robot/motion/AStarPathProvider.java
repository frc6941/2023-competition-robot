package frc.robot.motion;

import java.util.Optional;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.frcteam6941.pathplanning.astar.AStarPathPlanner;
import org.frcteam6941.pathplanning.smoothing.FloydTrimming;
import org.frcteam6941.pathplanning.universal.Path;

import edu.wpi.first.math.geometry.Translation2d;

public class AStarPathProvider implements PathProvider {
    private AStarPathPlanner planner = new AStarPathPlanner();
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private Future<Path> path = null;

    @Override
    public Optional<Path> getPath() {
        try {
            if(path != null) {
                return Optional.ofNullable( ( path.isDone() ? path.get() : null ) );
            } else {
                return Optional.empty();
            }
        } catch (Exception e) {
            System.out.println("Path Error." + e.toString());
            return Optional.empty();
        }
    }

    @Override
    public boolean buildPath(Translation2d startingPoint, Translation2d endingPoint) {
        if(!planner.checkLegal(startingPoint, FieldObstacles.obstacleList) && !planner.checkLegal(endingPoint, FieldObstacles.obstacleList)) {
            path = executor.submit(new Callable<Path>() {
                @Override
                public Path call() throws Exception {
                    return FloydTrimming.trimPath(planner.plan(
                    startingPoint, endingPoint,
                    FieldObstacles.obstacleList,
                    0.05, 0.05,
                    2.5, 1.0));
                }
            });
            return true;
        } else {
            return false;
        }
        
    }

    @Override
    public void clear() {
        if(path != null) {
            path.cancel(true);
        }
        path = null;
    }
}