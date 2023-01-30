package frc.robot.states;

public class ScoringAndLoadingState {
    public enum GAME_PIECE {
        CUBE,
        CONE
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
        DOUBLE_SUBSTATION_LEFT,
        DOUBLE_SUBSTATION_RIGHT
    }

    public enum LOADING_SIDE {
        NEAR,
        FAR,
        NONE
    }

    // User determined inputs
    public GAME_PIECE targetGamePiece = GAME_PIECE.CONE;
    public SCORING_ROW scoringRow = SCORING_ROW.HIGH;
    public SCORING_SIDE scoringSide = SCORING_SIDE.LEFT;
    public SCORING_GRID scoringGrid = SCORING_GRID.RIGHT;

    public LOADING_LOCATION loadingLocation = LOADING_LOCATION.DOUBLE_SUBSTATION_LEFT;
    public LOADING_SIDE loadingSide = LOADING_SIDE.NONE;

    public ScoringAndLoadingState(GAME_PIECE targetGamePiece, SCORING_ROW scoringRow, SCORING_SIDE scoringSide, SCORING_GRID scoringGrid, LOADING_LOCATION loadingLocation, LOADING_SIDE loadingSide) {
        this.targetGamePiece = targetGamePiece;
        this.scoringRow = scoringRow;
        this.scoringSide = scoringSide;
        this.scoringGrid = scoringGrid;
        this.loadingLocation = loadingLocation;
        this.loadingSide = loadingSide;
    }
    
}
