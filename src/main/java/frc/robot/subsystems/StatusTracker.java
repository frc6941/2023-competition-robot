package frc.robot.subsystems;

import java.nio.file.Paths;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utils.Lights;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

public class StatusTracker implements Updatable {
    // public AddressableLEDWrapper led = new AddressableLEDWrapper(
    //     Constants.LED_CONTROL.LED_PORT,
    //     Constants.LED_CONTROL.LED_LENGTH);

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
    }

    public StatusTrackerPeriodicIO mPeriodicIO= new StatusTrackerPeriodicIO();

    public NetworkTable matchInfoTable = NetworkTableInstance.getDefault().getTable("MatchInfo");
    public BooleanPublisher dsConnected = matchInfoTable.getBooleanTopic("dsConnected").publish();
    public DoublePublisher matchTime = matchInfoTable.getDoubleTopic("matchTime").publish();
    public IntegerPublisher matchNumber = matchInfoTable.getIntegerTopic("matchNumber").publish();
    public StringPublisher matchStage = matchInfoTable.getStringTopic("matchStage").publish();
    public StringPublisher matchName = matchInfoTable.getStringTopic("matchName").publish();
    public StringPublisher matchType = matchInfoTable.getStringTopic("matchType").publish();

    Javalin app = Javalin.create(
        config -> {
            config.staticFiles.add(
                Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(),"pulsehud").toString(), Location.EXTERNAL);
    }).start(3000);

    private static StatusTracker instance;

    private StatusTracker() {
        matchTime.setDefault(-1.0);
        matchStage.setDefault(MATCH_STAGE.DISCONNECTED.toString());
        matchName.setDefault("Test Match");
        matchNumber.setDefault(-1);
        dsConnected.setDefault(false);
        matchType.setDefault(MatchType.None.toString());
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

    public boolean inInScore() {
        return mPeriodicIO.isInScore;
    }

    public void setLoad() {
        mPeriodicIO.isInload = true;
        mPeriodicIO.isInScore = false;
    }

    public void setScore() {
        mPeriodicIO.isInload = false;
        mPeriodicIO.isInScore = true;
    }

    public void clear() {
        mPeriodicIO.isInload = false;
        mPeriodicIO.isInScore = false;
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.dsConnected = DriverStation.isDSAttached();

        if(mPeriodicIO.dsConnected) {
            mPeriodicIO.matchTime = DriverStation.getMatchTime();
            mPeriodicIO.matchName = DriverStation.getEventName();
            mPeriodicIO.matchNumber = DriverStation.getMatchNumber();
            mPeriodicIO.matchType = DriverStation.getMatchType();
            
            if(DriverStation.isEnabled()) {
                if(DriverStation.isAutonomousEnabled()) {
                    mPeriodicIO.matchStage = MATCH_STAGE.AUTO;
                } else if (DriverStation.isTeleopEnabled()) {
                    if(mPeriodicIO.matchTime > 30.0) {
                        mPeriodicIO.matchStage = MATCH_STAGE.TELEOP;
                    } else {
                        mPeriodicIO.matchStage = MATCH_STAGE.ENDGAME;
                    }
                }
            } else {
                if(DriverStation.isEStopped()) {
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
        // led.setPattern(mPeriodicIO.desiredPattern);
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
