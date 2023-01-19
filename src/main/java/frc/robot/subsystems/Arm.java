package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;

import com.team254.lib.drivers.LazyTalonFX;

import frc.robot.Constants;

public class Arm implements Updatable {
    public static class PeriodicIO {
        // INPUT
        public double armDemand = 0.0;
        public double armFeedforwardLength = 0.0;

        // OUTPUT
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazyTalonFX armMotor = new LazyTalonFX(Constants.CANID.ARM_MOTOR);

    @Override
    public synchronized void read(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void update(double time, double dt) {
        // Auto Generated Method
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
    public synchronized void disabled(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public void simulate() {

    }

    public enum STATE {
        ANGLE,
        PERCENTAGE,
        HOMING,
        OFF
    }

    private STATE state = STATE.HOMING;

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return state;
    }
}
