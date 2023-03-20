package frc.robot.states;

public class LoadingTarget {
    public static enum LOADING_LOCATION {
        GROUND_TIPPED(3),
        GROUND(2),
        SINGLE_SUBSTATION(1),
        DOUBLE_SUBSTATION(0);

        public int id;

        LOADING_LOCATION(int id) {
            this.id = id;
        }
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

    public LoadingTarget(int id) {
        switch(id) {
            case 0:
                this.loadingLocation = LOADING_LOCATION.DOUBLE_SUBSTATION;
                break;
            case 1:
                this.loadingLocation = LOADING_LOCATION.SINGLE_SUBSTATION;
                break;
            case 2:
                this.loadingLocation = LOADING_LOCATION.GROUND;
                break;
            case 3:
                this.loadingLocation = LOADING_LOCATION.GROUND_TIPPED;
                break;
        }
    }
}
