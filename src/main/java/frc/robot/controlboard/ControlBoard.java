package frc.robot.controlboard;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.controlboard.CustomButtonBoard.BUTTON;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Side;

public class ControlBoard {
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

    public void updateRumble(double time) {
        driver.updateRumble(time);
    }

    /**
     * DRIVER METHODS
     * 
     * LB - Cancel
     * RB - Confirm
     * LT - Brake
     * RT - Auto Path
     * 
     * Start - Reset Gyro
     * POV Left - Loading
     * POV Right - Scoring
     * Y - Arm Up
     * A - Arm Down
     * X - Intake
     * B - Outtake
     * 
*/
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);

        forwardAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);
        return tAxes;
    }

    public double getBrakeScale() {
        return 1.0 - driver.getTrigger(Side.LEFT) * Constants.CONTROLBOARD.DRIVER_BRAKE_MAX;
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X) * 1.0;
        rotAxis = Constants.CONTROLBOARD.CONTROLLER_INVERT_R ? rotAxis : -rotAxis;
        return rotAxis;
    }

    public boolean getConfirmation() {
        return driver.getController().getHID().getRightBumperPressed();
    }

    public boolean getCancellation() {
        return driver.getController().getHID().getLeftBumperPressed();
    }

    public boolean getForceExtendInScore() {
        return driver.getController().povUp().getAsBoolean();
    }

    public Trigger getResetGyro() {
        return driver.getController().start();
    }

    public Trigger getLoad() {
        return driver.getController().povLeft();
    }

    public Trigger getScore() {
        return driver.getController().povRight();
    }

    public Trigger getAutoPath() {
        return driver.getController().rightTrigger(0.5);
    }

    public Trigger getSpit() {
        return driver.getController().b();
    }

    public Trigger getIntake() {
        return driver.getController().x();
    }

    public Trigger getArmIncrease() {
        return driver.getController().y();
    }

    public Trigger getArmDecrease() {
        return driver.getController().a();
    }
    
    /**
     * OPERATOR METHODS
     */
    // public Trigger getTargetMoveLeft() {
    //     return operator.button(BUTTON.ML);
    // }

    // public Trigger getTargetMoveRight() {
    //     return operator.button(BUTTON.MR);
    // }

    // public Trigger getTargetMoveForward() {
    //     return operator.button(BUTTON.UM);
    // }

    // public Trigger getTargetMoveBackward() {
    //     return operator.button(BUTTON.LM);
    // }

    // public Trigger getApplyCursor() {
    //     return operator.button(BUTTON.MM);
    // }

    // public Trigger getLoadingStation() {
    //     return operator.button(BUTTON.UL);
    // }

    // public Trigger getGroundLoading() {
    //     return operator.button(BUTTON.UR);
    // }

    // public Trigger getSingleSubstation() {
    //     return operator.button(BUTTON.LL);
    // }

    // public Trigger getGroundTipped() {
    //     return operator.button(BUTTON.LR);
    // }

    // public Trigger getCanCommuteNear() {
    //     return operator.axisLessThan(1, -0.5);
    // }

    public Trigger getTargetMoveLeft() {
        return operator.getController().povLeft();
    }

    public Trigger getTargetMoveRight() {
        return operator.getController().povRight();
    }

    public Trigger getTargetMoveForward() {
        return operator.getController().povUp();
    }

    public Trigger getTargetMoveBackward() {
        return operator.getController().povDown();
    }

    public Trigger getLoadingTargetIncrease() {
        return operator.getController().rightBumper();
    }

    public Trigger getLoadingTargetDecrease() {
        return operator.getController().leftBumper();
    }

    public Trigger getApplyCursor() {
        return operator.getController().a();
    }

    public Trigger getCanCommuteNear() {
        return operator.getController().leftTrigger(0.5).and(
            () -> operator.getController().rightTrigger(0.5).getAsBoolean()
        );
    }
}
