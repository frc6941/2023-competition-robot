package org.frcteam6941.pathplanning.smoothing;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

public class FloydTrimming {
    public static ArrayList<Translation2d> trimPath(ArrayList<Translation2d> path) {
        // Handle inputs
        if (path == null) {
            return path;
        }

        int pathLength = path.size();
        if (pathLength > 2) {
            // Calculate the vector between -3 and -2 points, finding a direction
            Translation2d vector = path.get(pathLength - 1).minus(path.get(pathLength - 2));
            Translation2d tempVector;

            for (int i = pathLength - 3; i >= 0; i--) {
                // Calculate the vector of the -4 and -3 points
                tempVector = path.get(i + 1).minus(path.get(i));
                // Caculate cross product, if zero is produced then points are on the same
                // straight line, trim it
                if ((vector.getX() * tempVector.getY() - tempVector.getX() * vector.getY()) == 0) {
                    path.remove(i + 1);
                    // Else, update the prodcut to be the next direction indicator
                } else {
                    vector = tempVector;
                }
            }
        }
        return path;
    }
}
