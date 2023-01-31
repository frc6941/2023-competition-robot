package frc.robot.states;

public class ScoringTarget {
    public static enum SCORING_ROW {
        LOW,
        MID,
        HIGH
    }

    public static enum SCORING_GRID {
        OUTER(1),
        INNER(7),
        COOPERTITION(4);

        public int num;
        SCORING_GRID(int num) {
            this.num = num;
        }
    }

    // In the direction of driver.
    public static enum SCORING_SIDE {
        OUTER(-1),
        INNER(1);

        public int delta;
        SCORING_SIDE(int delta) {
            this.delta = delta;
        }
    }

    // User determined inputs
    private GamePiece targetGamePiece;
    private SCORING_ROW scoringRow;
    private SCORING_GRID scoringGrid;
    private SCORING_SIDE scoringSide;
    
    

    public ScoringTarget(GamePiece targetGamePiece, SCORING_ROW scoringRow, SCORING_GRID scoringGrid, SCORING_SIDE scoringSide) {
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

    public int getPosition() {
        if(targetGamePiece == GamePiece.CUBE) {
            return scoringGrid.num;
        } else if (targetGamePiece == GamePiece.CONE) {
            return scoringGrid.num + scoringSide.delta;
        } else {
            System.out.println("Scoring Target: Invalid Game Piece.");
            return -1;
        }
    }
}
