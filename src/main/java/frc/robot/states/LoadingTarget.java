package frc.robot.states;

public class LoadingTarget {
    public static enum LOADING_LOCATION {
        GROUND,
        GROUND_TIPPED,
        SINGLE_SUBSTATION,
        DOUBLE_SUBSTATION,
    }

    private LOADING_LOCATION loadingLocation;


    public LoadingTarget(LOADING_LOCATION loadingLocation) {
        this.loadingLocation = loadingLocation;
    }

    public LOADING_LOCATION getLoadingLocation() {
        return this.loadingLocation;
    }

    public void setLoadingLocation(LOADING_LOCATION loadingLocation) {
        this.loadingLocation = loadingLocation;
    }
}
