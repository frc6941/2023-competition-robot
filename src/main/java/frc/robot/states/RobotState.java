package frc.robot.states;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    /*
     * Different Modes of the robot.
     * 
     * SCORING: The robot is in front of the grid, and is 
     */
    public enum MODE {
        SCORING,
        COMMUTING,
        LOADING,
        DOCKING
    }

    public enum TARGET_LOCATION {
        GRID_LEFT,
        GRID_MIDDLE,
        GRID_RIGHT,
        DOUBLE_SUBSTATION
    }

    public enum TARGET_HEIGHT {
        LOW, MID, HIGH
    }

    public Alliance alliance;
    public TARGET_LOCATION location;
    public TARGET_HEIGHT height;

    public RobotState(MODE mode, Alliance alliance, TARGET_LOCATION location, TARGET_HEIGHT height) {
        this.alliance = alliance;
        this.location = location;
        this.height = height;
    }
}
