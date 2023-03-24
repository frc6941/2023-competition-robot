package org.frcteam6941.pathplanning.astar.obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class CircleObstacle implements Obstacle {
    private Translation2d center;
    private double radius;

    public CircleObstacle(Translation2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    @Override
    public boolean isInObstacle(double x, double y) {
        // A point is in a polygon if a line from the point to infinity crosses the
        // polygon an odd number of times
        return new Translation2d(x, y).minus(center).getNorm() < radius;
    }
}
