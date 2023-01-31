package frc.robot.states;

public class LoadingTarget {
    public static enum LOADING_LOCATION {
        GROUND,
        DOUBLE_SUBSTATION_OUTER,
        DOUBLE_SUBSTATION_INNER
    }

    public static enum LOADING_SIDE {
        NEAR,
        FAR,
        NONE
    }

    private GamePiece targetGamePiece;
    private LOADING_LOCATION loadingLocation;
    private LOADING_SIDE loadingSide;


    public LoadingTarget(GamePiece targetGamePiece, LOADING_LOCATION loadingLocation, LOADING_SIDE loadingSide) {
        this.targetGamePiece = targetGamePiece;
        this.loadingLocation = loadingLocation;
        this.loadingSide = loadingSide;
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

    public LOADING_SIDE getLoadingSide() {
        return this.loadingSide;
    }

    public void setLoadingSide(LOADING_SIDE loadingSide) {
        this.loadingSide = loadingSide;
    }
}
