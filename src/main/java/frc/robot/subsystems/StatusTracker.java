package frc.robot.subsystems;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.AddressableLEDWrapper;
import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.utils.Lights;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;

public class StatusTracker implements Updatable {
    // public AddressableLEDWrapper led = new AddressableLEDWrapper(
    //     Constants.LED_CONTROL.LED_PORT,
    //     Constants.LED_CONTROL.LED_LENGTH);

    public static class StatusTrackerPeriodicIO {
        public AddressableLEDPattern desiredPattern = Lights.CONNECTING;

        public boolean isInload = false;
        public boolean isInScore = false;

        public double matchTime = 0.0;
        public double matchNumber = 0;
        public boolean dsConnected = false;
        public String matchStage = "Autonomous";
        public String matchName = "Practice";
        public String matchType = "PRAC";
    }

    public StatusTrackerPeriodicIO mPeriodicIO= new StatusTrackerPeriodicIO();

    public NetworkTable matchInfoTable = NetworkTableInstance.getDefault().getTable("MatchInfo");
    public DoubleTopic matchTime = matchInfoTable.getDoubleTopic("matchTime");
    public StringTopic matchStage = matchInfoTable.getStringTopic("matchStage");
    public StringTopic matchName = matchInfoTable.getStringTopic("matchName");
    public StringTopic matchType = matchInfoTable.getStringTopic("matchType");

    Javalin app = Javalin.create(
        config -> {
            config.staticFiles.add(
                Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(),"pulsehud").toString(), Location.EXTERNAL);
    }).start(6941);

    private static StatusTracker instance;

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
            mPeriodicIO.matchType = DriverStation.getMatchType().toString();
            
            if(DriverStation.isEnabled()) {
                if(DriverStation.isAutonomousEnabled()) {
                    mPeriodicIO.matchStage = "Autonomus";
                } else if (DriverStation.isTeleopEnabled()) {
                    if(mPeriodicIO.matchTime > 30.0) {
                        mPeriodicIO.matchStage = "Teleop";
                    } else {
                        mPeriodicIO.matchStage = "EndGame";
                    }
                }
            } else {
                if(DriverStation.isEStopped()) {
                    mPeriodicIO.matchStage = "EStopped";
                } else {
                    mPeriodicIO.matchStage = "Disabled";
                }
            }
        } else {
            mPeriodicIO.matchTime = -1.0;
            mPeriodicIO.matchName = "Unknown";
            mPeriodicIO.matchNumber = -1;
            mPeriodicIO.matchType = "Unknown";
            mPeriodicIO.matchStage = "Disconnected";
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
        // Auto Generated Method
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
        // Auto Generated Method
    }
}
