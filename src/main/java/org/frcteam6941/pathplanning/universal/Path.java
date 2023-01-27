package org.frcteam6941.pathplanning.universal;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

public class Path {
    private double length;
    private ArrayList<Translation2d> pathPoints;


    public Path(double length, ArrayList<Translation2d> pathPoints) {
        this.length = length;
        this.pathPoints = pathPoints;
    }


    public double getLength() {
        return this.length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public ArrayList<Translation2d> getPathPoints() {
        return this.pathPoints;
    }

    public void setPathPoints(ArrayList<Translation2d> pathPoints) {
        this.pathPoints = pathPoints;
    }
    
}
