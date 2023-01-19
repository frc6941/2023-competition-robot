package frc.robot.states;

public class ScoringAndLoadingState {
    public enum GAME_PIECE {
        CUBE,
        CONE,
        NONE
    }

    public enum SCORING_ROW {
        LOW,
        MID,
        HIGH
    }

    public enum SCORING_GRID {
        LEFT,
        RIGHT,
        COOPERTITION
    }

    // In the direction of driver.
    public enum SCORING_SIDE {
        LEFT,
        RIGHT
    }

    public enum LOADING_LOCATION {
        GROUND,
        DOUBLE_SUBSTATION
    }

    public GAME_PIECE gamePiece = GAME_PIECE.NONE;
    public SCORING_ROW scoringRow = SCORING_ROW.HIGH;
    public SCORING_SIDE scoringSide = SCORING_SIDE.LEFT;
    public SCORING_GRID scoringGrid = SCORING_GRID.RIGHT;
    public LOADING_LOCATION location = LOADING_LOCATION.DOUBLE_SUBSTATION;
}
