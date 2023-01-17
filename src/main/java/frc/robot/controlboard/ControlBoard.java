package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Button;
import frc.robot.controlboard.CustomXboxController.Side;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;

public class ControlBoard {
    public final double kSwerveDeadband = Constants.CONTROLBOARD.CONTROLLER_DEADBAND;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final CustomXboxController driver;
    private final CustomXboxController operator;

    private ControlBoard() {
        driver = new CustomXboxController(Constants.CONTROLBOARD.DRIVER_CONTROLLER_PORT);
        operator = new CustomXboxController(Constants.CONTROLBOARD.OPERATOR_CONTROLLER_PORT);
    }

    public CustomXboxController getDriverController() {
        return driver;
    }

    public CustomXboxController getOperatorController() {
        return operator;
    }

    public void setDriverRumble(double power, double interval) {
        driver.setRumble(power, interval);
    }

    public void setOperatorRumble(double power, double interval) {
        operator.setRumble(power, interval);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);
        double pedal = driver.getTrigger(Side.RIGHT);
        double breaker = driver.getTrigger(Side.LEFT);

        forwardAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            double pedalScale = 1.0 - Constants.CONTROLBOARD.CONTROLLER_PEDAL + Constants.CONTROLBOARD.CONTROLLER_PEDAL * pedal;
            double breakScale = (1.0 - Constants.CONTROLBOARD.CONTROLLER_PEDAL) * breaker;
            return tAxes.times(pedalScale).minus(tAxes.times(breakScale)).times(Constants.SUBSYSTEM_SWERVE.DRIVE_MAX_VELOCITY);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X) * 2.0;
        rotAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean getSwerveBrake() {
        return driver.getButton(Button.R_JOYSTICK);
    }

    public boolean zeroGyro() {
        return driver.getController().getStartButtonPressed();
    }

    public SWERVE_CARDINAL getSwerveSnapRotation() {
        if (driver.getButton(Button.A)) {
            return SWERVE_CARDINAL.BACKWARDS;
        } else if (driver.getButton(Button.X)) {
            return SWERVE_CARDINAL.RIGHT;
        } else if (driver.getButton(Button.B)) {
            return SWERVE_CARDINAL.LEFT;
        } else if (driver.getButton(Button.Y)) {
            return SWERVE_CARDINAL.FORWARDS;
        } else {
            return SWERVE_CARDINAL.NONE;
        }
    }    
}