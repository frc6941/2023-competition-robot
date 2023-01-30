package frc.robot.states;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Targets {
    public static CoordinatedTarget getLoadingCoordinatedTarget(ScoringAndLoadingState state) {
        switch(state.targetGamePiece) {
            case CONE:
                break;
            case CUBE:
                break;
        }

        switch(state.loadingLocation) {
            case DOUBLE_SUBSTATION_LEFT:
                break;
            case DOUBLE_SUBSTATION_RIGHT:
                break;
            case GROUND:
                break;
        }
        return null;
    }
}
