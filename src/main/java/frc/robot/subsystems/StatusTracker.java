package frc.robot.subsystems;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.AddressableLEDWrapper;
import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.Constants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.states.GamePiece;
import frc.robot.utils.Lights;

public class StatusTracker implements Updatable {
    public AddressableLEDWrapper led;

    public static class StatusTrackerPeriodicIO {
        public AddressableLEDPattern desiredPattern = Lights.CONNECTING;

        public boolean isInload = false;
        public boolean isInScore = false;

        public double matchTime = -1.0;
        public int matchNumber = -1;
        public boolean dsConnected = false;
        public MATCH_STAGE matchStage = MATCH_STAGE.DISCONNECTED;
        public String matchName = "Test Match";
        public MatchType matchType = MatchType.None;
        public Alliance matchAlliance = Alliance.Invalid;

        public boolean seeAprilTag = false;
        public boolean hasGamePiece = false;
        public boolean isCube = false;
        public boolean inManual = false;
        public boolean speedLimitActivate = false;
        public boolean yCancelActivate = false;
        public boolean forceExtend = false;
        public boolean hasLoadChanged = false;
    }

    public StatusTrackerPeriodicIO mPeriodicIO = new StatusTrackerPeriodicIO();

    public NetworkTable matchInfoTable = NetworkTableInstance.getDefault().getTable("MatchInfo");
    public BooleanPublisher dsConnected = matchInfoTable.getBooleanTopic("dsConnected").publish();
    public DoublePublisher matchTime = matchInfoTable.getDoubleTopic("matchTime").publish();
    public IntegerPublisher matchNumber = matchInfoTable.getIntegerTopic("matchNumber").publish();
    public StringPublisher matchStage = matchInfoTable.getStringTopic("matchStage").publish();
    public StringPublisher matchName = matchInfoTable.getStringTopic("matchName").publish();
    public StringPublisher matchType = matchInfoTable.getStringTopic("matchType").publish();
    public StringPublisher matchAlliance = matchInfoTable.getStringTopic("matchAlliance").publish();

    // Javalin app = Javalin.create(
    //         config -> {
    //             config.staticFiles.add(
    //                     Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "pulsehud").toString(),
    //                     Location.EXTERNAL);
    //         }).start(5807);

    private static StatusTracker instance;

    private StatusTracker() {
        matchTime.setDefault(-1.0);
        matchStage.setDefault(MATCH_STAGE.DISCONNECTED.toString());
        matchName.setDefault("Test Match");
        matchNumber.setDefault(-1);
        dsConnected.setDefault(false);
        matchType.setDefault(MatchType.None.toString());

        led = new AddressableLEDWrapper(
            Constants.LED_CONTROL.LED_PORT,
            Constants.LED_CONTROL.LED_LENGTH);
        led.setIntensity(1.0);
        led.start(0.02);
    }

    public static StatusTracker getInstance() {
        if (instance == null) {
            instance = new StatusTracker();
        }
        return instance;
    }

    public boolean isInLoad() {
        return mPeriodicIO.isInload;
    }

    public boolean isInScore() {
        return mPeriodicIO.isInScore;
    }

    public boolean isSpeedRedctionActivate() {
        return mPeriodicIO.speedLimitActivate;
    }

    public boolean isInManual() {
        return mPeriodicIO.inManual;
    }

    public boolean isYCancelActivate() {
        return mPeriodicIO.yCancelActivate;
    }


    public void setInManual(boolean value) {
        mPeriodicIO.inManual = value;
    }

    public void setForceExtend(boolean value) {
        mPeriodicIO.forceExtend = value;
    }

    public void toggleForceExtend() {
        mPeriodicIO.forceExtend = !mPeriodicIO.forceExtend;
    }

    public boolean getForceExtend() {
        return mPeriodicIO.forceExtend;
    }

    public void setYCancel(boolean value) {
        mPeriodicIO.yCancelActivate = value;
    }

    public boolean hasLoadHasChanged() {
        return mPeriodicIO.hasLoadChanged;
    }

    public void setLoadHasChanged() {
        mPeriodicIO.hasLoadChanged = true;
    }

    public void clearLoadRecorder() {
        mPeriodicIO.hasLoadChanged = false;
    }

    public void setLoad() {
        mPeriodicIO.isInload = true;
        mPeriodicIO.isInScore = false;
    }

    public void setScore() {
        mPeriodicIO.isInload = false;
        mPeriodicIO.isInScore = true;
    }

    public void enableSpeedLimit() {
        mPeriodicIO.speedLimitActivate = true;
    }

    public void clear() {
        mPeriodicIO.isInload = false;
        mPeriodicIO.isInScore = false;
        mPeriodicIO.speedLimitActivate = false;
        mPeriodicIO.yCancelActivate = false;
    }

    public void updateIndicator() {
        switch (mPeriodicIO.matchStage) {
            case PREP:
                if (mPeriodicIO.matchAlliance == Alliance.Red) {
                    mPeriodicIO.desiredPattern = Lights.ALLIANCE_RED;
                } else if (mPeriodicIO.matchAlliance == Alliance.Blue) {
                    mPeriodicIO.desiredPattern = Lights.ALLIANCE_BLUE;
                } else {
                    mPeriodicIO.desiredPattern = Lights.CONNECTING;
                }
                break;
            case AUTO:
                mPeriodicIO.desiredPattern = Lights.AUTO_SHOW;
                break;
            case TELEOP:
            case ENDGAME:
                if (mPeriodicIO.inManual) {
                    mPeriodicIO.desiredPattern = Lights.MANUAL;
                } else if (mPeriodicIO.isInScore) {
                    switch(TargetSelector.getInstance().getScoringRow()) {
                        case HIGH:
                            mPeriodicIO.desiredPattern = Lights.SCORING_HIGH;
                            break;
                        case MID:
                            mPeriodicIO.desiredPattern = Lights.SCORING_MID;
                            break;
                        case LOW:
                        default:
                            mPeriodicIO.desiredPattern = Lights.SCORING_LOW;
                            break;
                    }
                } else if (mPeriodicIO.isInload) {
                    if(mPeriodicIO.hasGamePiece) {
                        mPeriodicIO.desiredPattern = Lights.HAS_GAMEPIECE;
                    } else {
                        mPeriodicIO.desiredPattern = mPeriodicIO.isCube ? Lights.LOAD_CUBE : Lights.LOAD_CONE;
                    }
                } else {
                    switch(TargetSelector.getInstance().getScoringRow()) {
                        case HIGH:
                            mPeriodicIO.desiredPattern = mPeriodicIO.isCube ? Lights.COMMUTE_CUBE_HIGH : Lights.COMMUTE_CONE_HIGH;
                            break;
                        case MID:
                            mPeriodicIO.desiredPattern = mPeriodicIO.isCube ? Lights.COMMUTE_CUBE_MID : Lights.COMMUTE_CONE_MID;
                            break;
                        case LOW:
                        default:
                            mPeriodicIO.desiredPattern = mPeriodicIO.isCube ? Lights.COMMUTE_CUBE_LOW : Lights.COMMUTE_CONE_LOW;
                            break;
                    }
                }
                break;
            case ESTOP:
                mPeriodicIO.desiredPattern = Lights.ESTOP;
                break;
            case DISCONNECTED:
                mPeriodicIO.desiredPattern = Lights.CONNECTING;
                break;
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.dsConnected = DriverStation.isDSAttached();
        mPeriodicIO.matchAlliance = DriverStation.getAlliance();
        mPeriodicIO.seeAprilTag = RobotStateEstimator.getInstance().seeAprilTag();
        mPeriodicIO.isCube = TargetSelector.getInstance().getTargetGamePiece() == GamePiece.CUBE;
        mPeriodicIO.hasGamePiece = Intaker.getInstance().hasGamePiece();

        if (mPeriodicIO.dsConnected) {
            mPeriodicIO.matchTime = DriverStation.getMatchTime();
            if (DriverStation.getEventName().length() == 0) {
                mPeriodicIO.matchName = "Unknown";
            } else {
                mPeriodicIO.matchName = DriverStation.getEventName();
            }
            mPeriodicIO.matchNumber = DriverStation.getMatchNumber();
            mPeriodicIO.matchType = DriverStation.getMatchType();

            if (DriverStation.isEnabled()) {
                if (DriverStation.isAutonomousEnabled()) {
                    mPeriodicIO.matchStage = MATCH_STAGE.AUTO;
                } else if (DriverStation.isTeleopEnabled()) {
                    if (mPeriodicIO.matchTime > 30.0 || mPeriodicIO.matchTime < 0.0) {
                        mPeriodicIO.matchStage = MATCH_STAGE.TELEOP;
                    } else {
                        mPeriodicIO.matchStage = MATCH_STAGE.ENDGAME;
                    }
                }
            } else {
                if (DriverStation.isEStopped()) {
                    mPeriodicIO.matchStage = MATCH_STAGE.ESTOP;
                } else {
                    mPeriodicIO.matchStage = MATCH_STAGE.PREP;
                }
            }
        } else {
            mPeriodicIO.matchTime = -1.0;
            mPeriodicIO.matchName = "Unknown";
            mPeriodicIO.matchNumber = -1;
            mPeriodicIO.matchType = MatchType.None;
            mPeriodicIO.matchStage = MATCH_STAGE.DISCONNECTED;
        }
    }

    @Override
    public synchronized void update(double time, double dt) {
        updateIndicator();
        ControlBoard.getInstance().updateRumble(time);
        led.setPattern(mPeriodicIO.desiredPattern);
    }

    @Override
    public synchronized void write(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void telemetry() {
        matchTime.set(mPeriodicIO.matchTime);
        matchStage.set(mPeriodicIO.matchStage.toString());
        matchName.set(mPeriodicIO.matchName);
        matchNumber.set(mPeriodicIO.matchNumber);
        dsConnected.set(mPeriodicIO.dsConnected);
        matchType.set(mPeriodicIO.matchType.toString());
        matchAlliance.set(mPeriodicIO.matchAlliance.toString());
    }

    @Override
    public synchronized void start() {
        // Auto Generated Method
    }

    @Override
    public synchronized void stop() {
        // Auto Generated Method
    }

    @Override
    public synchronized void simulate(double time, double dt) {
        read(time, dt);
    }

    public enum MATCH_STAGE {
        PREP,
        AUTO,
        TELEOP,
        ENDGAME,
        ESTOP,
        DISCONNECTED
    }
}
