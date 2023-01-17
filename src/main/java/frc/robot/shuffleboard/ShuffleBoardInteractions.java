package frc.robot.shuffleboard;

import java.util.ArrayList;

public class ShuffleBoardInteractions {

    // Trims unnecessary tabs when in competition
    public final boolean mDebug = true;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private final ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<>();


    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        for (ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
    }
}