package frc.robot.states;

public class LoadingTarget {
    public static enum LOADING_LOCATION {
        GROUND,
        DOUBLE_SUBSTATION_OUTER,
        DOUBLE_SUBSTATION_INNER
    }

    private GamePiece targetGamePiece;
    private LOADING_LOCATION loadingLocation;


    public LoadingTarget(GamePiece targetGamePiece, LOADING_LOCATION loadingLocation) {
        this.targetGamePiece = targetGamePiece;
        this.loadingLocation = loadingLocation;
    }

    public GamePiece getTargetGamePiece() {
        return this.targetGamePiece;
    }

    public void setTargetGamePiece(GamePiece targetGamePiece) {
        this.targetGamePiece = targetGamePiece;
    }

    public LOADING_LOCATION getLoadingLocation() {
        return this.loadingLocation;
    }

    public void setLoadingLocation(LOADING_LOCATION loadingLocation) {
        this.loadingLocation = loadingLocation;
    }
}
