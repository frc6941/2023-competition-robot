package frc.robot.controlboard;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.controlboard.CustomButtonBoard.BUTTON;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Side;

public class ControlBoard {
    public final double kSwerveDeadband = Constants.CONTROLBOARD.CONTROLLER_DEADBAND;
    private static ControlBoard instance = null;
    private SlewRateLimiter xLimiter = new SlewRateLimiter(5.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final CustomXboxController driver;
    private final CustomButtonBoard operator;

    private ControlBoard() {
        driver = new CustomXboxController(Constants.CONTROLBOARD.DRIVER_CONTROLLER_PORT);
        operator = new CustomButtonBoard(Constants.CONTROLBOARD.OPERATOR_CONTROLLER_PORT);
    }

    public CustomXboxController getDriverController() {
        return driver;
    }

    public CustomButtonBoard getOperatorController() {
        return operator;
    }

    public void setDriverRumble(double power, double interval) {
        driver.setRumble(power, interval);
    }

    public void updateRumble(double time) {
        driver.updateRumble(time);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = xLimiter.calculate(driver.getAxis(Side.LEFT, Axis.Y));
        double strafeAxis = yLimiter.calculate(driver.getAxis(Side.LEFT, Axis.X));
        // double strafeAxis = 0.0;
        double pedal = driver.getTrigger(Side.RIGHT);

        forwardAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            double pedalScale = 1.0 - Constants.CONTROLBOARD.CONTROLLER_PEDAL + Constants.CONTROLBOARD.CONTROLLER_PEDAL * pedal;
            return tAxes.times(pedalScale);
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

    public boolean getConfirmation() {
        return driver.getController().getHID().getAButtonPressed();
    }

    public boolean getCancellation() {
        return driver.getController().getHID().getBButtonPressed();
    }

    public boolean getTargetMoveRight() {
        return operator.getRawButtonPressed(BUTTON.MR);
    }

    public boolean getTargetMoveLeft() {
        return operator.getRawButtonPressed(BUTTON.ML);
    }

    public boolean getTargetMoveUp() {
        return operator.getRawButtonPressed(BUTTON.UM);
    }

    public boolean getTargetMoveDown() {
        return operator.getRawButtonPressed(BUTTON.LM);
    }

    public boolean getLoadStation() {
        return operator.getRawButtonPressed(BUTTON.UL);
    }

    public boolean getLoadGround() {
        return operator.getRawButtonPressed(BUTTON.UR);
    }

    public double getSweeperExtension() {
        return driver.getTrigger(Side.LEFT);
    }
}
