package frc.robot.auto;

import java.util.Optional;

import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.MobilityAuto;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class AutoSelector {
    public enum AUTO_MODES {
        INNER_CONE_CONE,
        MOBILITY
    }

    private AUTO_MODES mCachedDesiredMode = AUTO_MODES.MOBILITY;

    private AutoModeBase mAutoMode;

    private final SendableChooser<AUTO_MODES> mModeChooser;

    private boolean autoWarning = false;

    private AutoSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Mobility", AUTO_MODES.MOBILITY);
        mModeChooser.addOption("Inner Cone and Cone", AUTO_MODES.INNER_CONE_CONE);
    }

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private static AutoSelector instance;

    public void updateModeCreator() {
        AUTO_MODES desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = AUTO_MODES.MOBILITY;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto Selection Changed:" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode).map(autoModeBase -> {
                resetStartingPosition(autoModeBase.getStartingPose());
                return autoModeBase;
            }).orElseGet(() -> {
                resetStartingPosition(new Pose2d());
                return null;
            });
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(AUTO_MODES mode) {
        switch (mode) {
            case MOBILITY:
                autoWarning = false;
                return Optional.of(
                    new MobilityAuto()
                );

            default:
                autoWarning = true;
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break;
        }

        autoWarning = true;
        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = null;
        mCachedDesiredMode = null;
    }

    public void resetStartingPosition(Pose2d pose) {
        SJTUSwerveMK5Drivebase.getInstance().resetPose(pose);
    }

    public Optional<AutoModeBase> getAutoMode() {
        return Optional.ofNullable(mAutoMode);
    }

    public SendableChooser<AUTO_MODES> getSendableChooser() {
        return mModeChooser;
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }
}
