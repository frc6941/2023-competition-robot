package frc.robot.states;

public class ScoringTarget {
    public static enum SCORING_ROW {
        LOW(0),
        MID(1),
        HIGH(2);

        public int id;
        SCORING_ROW(int id) {
            this.id = id;
        }
    }

    public static enum SCORING_GRID {
        OUTER(1),
        INNER(7),
        COOPERTITION(4);

        public int id;
        SCORING_GRID(int id) {
            this.id = id;
        }
    }

    // In the direction of driver.
    public static enum SCORING_SIDE {
        OUTER(-1),
        INNER(1),
        MIDDLE(0);

        public int delta;
        SCORING_SIDE(int delta) {
            this.delta = delta;
        }
    }

    // User determined inputs
    private SCORING_ROW scoringRow;
    private SCORING_GRID scoringGrid;
    private SCORING_SIDE scoringSide;

    public ScoringTarget(SCORING_ROW scoringRow, SCORING_GRID scoringGrid, SCORING_SIDE scoringSide) {
        this.scoringRow = scoringRow;
        this.scoringSide = scoringSide;
        this.scoringGrid = scoringGrid;
    }
    
    /*
     * [ Row, Column ]
     */
    public ScoringTarget(int[] ids) {
        int reference;
        if(ids[1] >= 0 && ids[1] <= 2) {
            this.scoringGrid = SCORING_GRID.OUTER;
            reference = 1;
        } else if(ids[1] >= 3 && ids[1] <= 5) {
            this.scoringGrid = SCORING_GRID.COOPERTITION;
            reference = 4;
        } else if(ids[1] >= 6 && ids[1] <= 8) {
            this.scoringGrid = SCORING_GRID.INNER;
            reference = 7;
        } else {
            System.out.println("Warning: Invalid Node Choice.");
            reference = -6941;
        }

        int delta = ids[1] - reference;
        switch(delta) {
            case 1:
                this.scoringSide = SCORING_SIDE.INNER;
                break;
            case -1:
                this.scoringSide = SCORING_SIDE.OUTER;
                break;
            case 0:
                this.scoringSide = SCORING_SIDE.MIDDLE;
                break;
            default:
                System.out.println("Warning: Invalid Node Choice.");
                break;
        }

        switch(ids[0]) {
            case 0:
                this.scoringRow = SCORING_ROW.LOW;
                break;
            case 1:
                this.scoringRow = SCORING_ROW.MID;
                break;
            case 2:
                this.scoringRow = SCORING_ROW.HIGH;
                break;
            default:
                System.out.println("Warning: Invalid Node Choice.");
                break;
        }
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
        return scoringGrid.id + scoringSide.delta;
    }

    public int[] getTargetArray() {
        return new int[] {
            this.scoringRow.id,
            getPosition()
        };
    }

    @Override
    public String toString() {
        String pattern = "Scoring Target (%s): %s, %s, %s";
        return String.format(pattern, getPosition(), scoringRow.toString(), scoringGrid.toString(), scoringSide.toString());
    }
}
