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

    public void updateRumble(double time) {
        driver.updateRumble(time);
        operator.updateRumble(time);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);
        // double strafeAxis = 0.0;
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
            return tAxes.times(pedalScale).minus(tAxes.times(breakScale));
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X) * 1.0;
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
        int POV = driver.getController().getPOV();
        if (POV == kDpadDown) {
            return SWERVE_CARDINAL.BACKWARDS;
        } else if (POV == kDpadRight) {
            return SWERVE_CARDINAL.RIGHT;
        } else if (POV == kDpadLeft) {
            return SWERVE_CARDINAL.LEFT;
        } else if (POV == kDpadUp) {
            return SWERVE_CARDINAL.FORWARDS;
        } else {
            return SWERVE_CARDINAL.NONE;
        }
    }

    public boolean getCommutePressed() {
        return driver.getController().getXButtonPressed();
    }

    public boolean getLoadPressed() {
        return driver.getController().getYButtonPressed();
    }

    public boolean getPrepScorePressed() {
        return driver.getController().getBButtonPressed();
    }

    public boolean getScorePressed() {
        return driver.getController().getAButtonPressed();
    }

    public boolean getAutoTracking() {
        return driver.getController().getRightBumperPressed() || driver.getController().getRightBumperReleased();
    }

    public boolean getAutoTrackingContinuous() {
        return driver.getController().getRightBumper();
    }



    /* OPERATOR METHODS */
    public boolean getWantManual() {
        return operator.getController().getAButtonPressed();
    }

    public boolean getExitManual() {
        return operator.getController().getBButtonPressed();
    }

    public boolean getManualWantAngleIncrease() {
        return operator.getController().getPOV() == kDpadLeft;
    }

    public boolean getManualWantAngleDecrease() {
        return operator.getController().getPOV() == kDpadRight;
    }

    public boolean getManualWantLengthIncrease() {
        return operator.getController().getPOV() == kDpadUp;
    }

    public boolean getManualWantLengthDecrease() {
        return operator.getController().getPOV() == kDpadDown;
    }

    public double getManualIntakerPercentage() {
        if(operator.getController().getXButton()) {
            return Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE;
        } else if (operator.getController().getYButton()) {
            return Constants.SUBSYSTEM_INTAKE.OUTTAKING_PERCENTAGE;
        } else {
            return 0.0;
        }
    }
}
