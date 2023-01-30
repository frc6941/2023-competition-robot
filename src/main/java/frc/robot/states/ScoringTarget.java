package frc.robot.states;

public class ScoringTarget {
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

    // User determined inputs
    private GamePiece targetGamePiece;
    private SCORING_ROW scoringRow;
    private SCORING_SIDE scoringSide;
    private SCORING_GRID scoringGrid;
    

    public ScoringTarget(GamePiece targetGamePiece, SCORING_ROW scoringRow, SCORING_SIDE scoringSide, SCORING_GRID scoringGrid) {
        this.targetGamePiece = targetGamePiece;
        this.scoringRow = scoringRow;
        this.scoringSide = scoringSide;
        this.scoringGrid = scoringGrid;
    }

    public GamePiece getTargetGamePiece() {
        return this.targetGamePiece;
    }

    public void setTargetGamePiece(GamePiece targetGamePiece) {
        this.targetGamePiece = targetGamePiece;
    }

    public SCORING_ROW getScoringRow() {
        return this.scoringRow;
    }

    public void setScoringRow(SCORING_ROW scoringRow) {
        this.scoringRow = scoringRow;
    }

    public SCORING_SIDE getScoringSide() {
        return this.scoringSide;
    }

    public void setScoringSide(SCORING_SIDE scoringSide) {
        this.scoringSide = scoringSide;
    }

    public SCORING_GRID getScoringGrid() {
        return this.scoringGrid;
    }

    public void setScoringGrid(SCORING_GRID scoringGrid) {
        this.scoringGrid = scoringGrid;
    }

}
