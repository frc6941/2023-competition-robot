package frc.robot.auto;

import java.util.Optional;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.AutoModeBase;

public class AutoSelector {
    private AutoModeBase mAutoMode;
    private final SendableChooser<AutoModeBase> mModeChooser;
    private boolean autoWarning = false;

    private AutoSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", null);
    }

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private static AutoSelector instance;

    public void updateModeCreator() {
        AutoModeBase tempMode = mModeChooser.getSelected();
        if (mAutoMode != tempMode && tempMode != null) {
            resetStartingPosition(tempMode.getStartingPose());
        }
        mAutoMode = mModeChooser.getSelected();
    }

    public void reset() {
        mAutoMode = null;
    }

    public void resetStartingPosition(Pose2d pose) {
        SJTUSwerveMK5Drivebase.getInstance().resetPose(pose);
    }

    public Optional<AutoModeBase> getAutoMode() {
        return Optional.ofNullable(mAutoMode);
    }

    public SendableChooser<AutoModeBase> getSendableChooser() {
        return mModeChooser;
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }
}
