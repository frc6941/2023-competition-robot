package frc.robot.subsystems;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.team254.lib.drivers.LazyTalonFX;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.states.SuperstructureState;

public class ArmAndExtender implements Updatable {
    @AutoLog
    public static class ArmAndExtenderPeriodicIO {
        // INPUT
        public double armCurrent = 0.0;
        public double armVoltage = 0.0;
        public double armTemperature = 0.0;
        public double armAngle = 0.0;
        public double armTrajectoryVelocitySetPoint = 0.0;

        public double extenderCurrent = 0.0;
        public double extenderVoltage = 0.0;
        public double extenderTemperature = 0.0;
        public double extenderLength = 0.80;

        // OUTPUT
        public double armDemand = 0.0;
        public double armFeedforward = 0.0;
        public double extenderDemand = 0.0;
        public double extenderFeedforward = 0.0;
    }

    public ArmAndExtenderPeriodicIOAutoLogged mPeriodicIO = new ArmAndExtenderPeriodicIOAutoLogged();

    private final LazyTalonFX armMotorLeader = new LazyTalonFX(Constants.CANID.ARM_MOTOR_LEADER);
    private final LazyTalonFX armMotorFollower = new LazyTalonFX(Constants.CANID.ARM_MOTOR_FOLLOWER);
    private final LazyTalonFX extenderMotor = new LazyTalonFX(Constants.CANID.EXTENDER_MOTOR);

    private final Mechanism2d mechDrawing = new Mechanism2d(4, 2.5);
    private final MechanismRoot2d mechRoot = mechDrawing.getRoot(
            "High Pivot", Constants.SUBSYSTEM_SUPERSTRUCTURE.STRUCTURE.HIGH_PIVOT_2D_LOCATION.getX(),
            Constants.SUBSYSTEM_SUPERSTRUCTURE.STRUCTURE.HIGH_PIVOT_2D_LOCATION.getY());
    private final MechanismLigament2d armMech = mechRoot
            .append(new MechanismLigament2d("Arm", 0.80, 0.0, 3, new Color8Bit(Color.kPurple)));

    private boolean armIsHomed = false;
    private boolean extenderIsHomed = false;

    private static ArmAndExtender instance;

    public static ArmAndExtender getInstance() {
        if (instance == null) {
            instance = new ArmAndExtender();
        }
        return instance;
    }

    private ArmAndExtender() {
        armMotorLeader.configFactoryDefault(50);
        armMotorLeader.config_kP(0, Constants.SUBSYSTEM_ARM.KP, 100);
        armMotorLeader.config_kI(0, Constants.SUBSYSTEM_ARM.KI, 100);
        armMotorLeader.config_kD(0, Constants.SUBSYSTEM_ARM.KD, 100);
        armMotorLeader.config_kF(0, Constants.SUBSYSTEM_ARM.KF, 100);
        armMotorLeader.configMotionCruiseVelocity(Constants.SUBSYSTEM_ARM.CRUISE_V, 100);
        armMotorLeader.configMotionAcceleration(Constants.SUBSYSTEM_ARM.CRUIVE_ACC, 100);

        extenderMotor.configFactoryDefault(50);
        extenderMotor.config_kP(0, Constants.SUBSYSTEM_ARM.KP, 100);
        extenderMotor.config_kI(0, Constants.SUBSYSTEM_ARM.KI, 100);
        extenderMotor.config_kD(0, Constants.SUBSYSTEM_ARM.KD, 100);
        extenderMotor.config_kF(0, Constants.SUBSYSTEM_ARM.KF, 100);
        extenderMotor.configMotionCruiseVelocity(Constants.SUBSYSTEM_ARM.CRUISE_V, 100);
        extenderMotor.configMotionAcceleration(Constants.SUBSYSTEM_ARM.CRUIVE_ACC, 100);
    }

    public double getAngle() {
        return mPeriodicIO.armAngle;
    }

    public double getLength() {
        return mPeriodicIO.extenderLength;
    }

    private void setAngle(double armAngle) {
        if (armState != ARM_STATE.HOMING) {
            armState = ARM_STATE.ANGLE;
            mPeriodicIO.armDemand = Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.ARM_RANGE.clamp(armAngle);
        }
    }

    private void setLength(double extenderLength) {
        if (extenderState != EXTENDER_STATE.HOMING) {
            extenderState = EXTENDER_STATE.LENGTH;
            mPeriodicIO.extenderDemand = Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE
                    .clamp(extenderLength);
        }
    }

    public void setSuperstructureState(SuperstructureState superstructureState) {
        SuperstructureState constrainedState = Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.SUPERSTRUCTURE_LIMIT
                .optimize(superstructureState, getSuperstructureState());
        setAngle(constrainedState.armAngle.getDegrees());
        setLength(constrainedState.extenderLength);
    }

    public SuperstructureState getSuperstructureState() {
        return new SuperstructureState(Rotation2d.fromDegrees(getAngle()), getLength());
    }

    public void homeArm(double homingAngle) {
        armMotorLeader.setSelectedSensorPosition(
                Conversions.degreesToFalcon(homingAngle, Constants.SUBSYSTEM_ARM.GEAR_RATIO));
        armIsHomed = true;
    }

    public void homeExtender(double homingLength) {
        extenderMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(
                        mPeriodicIO.extenderDemand / Constants.SUBSYSTEM_EXTENDER.WHEEL_CIRCUMFERENCE * 360.0,
                        Constants.SUBSYSTEM_EXTENDER.GEAR_RATIO));
        extenderIsHomed = true;
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.armAngle = Conversions.falconToDegrees(armMotorLeader.getSelectedSensorPosition(),
                Constants.SUBSYSTEM_ARM.GEAR_RATIO);
        mPeriodicIO.armCurrent = armMotorLeader.getSupplyCurrent();
        mPeriodicIO.armVoltage = armMotorLeader.getMotorOutputVoltage();
        mPeriodicIO.armTemperature = armMotorLeader.getTemperature();
        mPeriodicIO.armTrajectoryVelocitySetPoint = Units
                .rotationsPerMinuteToRadiansPerSecond(
                        Conversions.falconToRPM(armMotorLeader.getActiveTrajectoryVelocity(),
                                Constants.SUBSYSTEM_ARM.GEAR_RATIO));

        mPeriodicIO.extenderLength = Conversions.falconToDegrees(extenderMotor.getSelectedSensorPosition(),
                Constants.SUBSYSTEM_EXTENDER.GEAR_RATIO) / 360.0
                * Constants.SUBSYSTEM_EXTENDER.WHEEL_CIRCUMFERENCE;
        mPeriodicIO.extenderCurrent = extenderMotor.getSupplyCurrent();
        mPeriodicIO.extenderVoltage = extenderMotor.getMotorOutputVoltage();
        mPeriodicIO.extenderTemperature = extenderMotor.getTemperature();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (armMotorLeader.isFwdLimitSwitchClosed() == 1) {
            homeArm(Constants.SUBSYSTEM_ARM.HOME_ANGLE);
        }

        if (extenderMotor.isRevLimitSwitchClosed() == 1) {
            homeExtender(Constants.SUBSYSTEM_EXTENDER.HOME_LENGTH);
        }

        if (!armIsHomed) {
            armState = ARM_STATE.HOMING;
        }

        if (!extenderIsHomed) {
            extenderState = EXTENDER_STATE.HOMING;
        }

        // Determine Outputs
        switch (armState) {
            case HOMING:
                mPeriodicIO.armDemand = 0.2;
                mPeriodicIO.armFeedforward = 0.0;
                break;
            case ANGLE:
                mPeriodicIO.armDemand = Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.ARM_RANGE
                        .clamp(mPeriodicIO.armDemand);
                mPeriodicIO.armFeedforward = Constants.SUBSYSTEM_ARM.FEEDFORWARD.calculate(
                        Units.degreesToRadians(getAngle()), mPeriodicIO.armTrajectoryVelocitySetPoint) / 12.0;
                break;
            case PERCENTAGE:
                mPeriodicIO.armDemand = Util.clamp(mPeriodicIO.armDemand, -1.0, 1.0);
                mPeriodicIO.armFeedforward = 0.0;
                break;
            default:
                mPeriodicIO.armDemand = 0.0;
                break;
        }

        switch (extenderState) {
            case HOMING:
                mPeriodicIO.extenderDemand = -0.2;
                mPeriodicIO.extenderFeedforward = 0.0;
                break;
            case LENGTH:
                mPeriodicIO.extenderDemand = Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE
                        .clamp(mPeriodicIO.extenderDemand);
                mPeriodicIO.extenderFeedforward = 0.0;
                break;
            case PERCENTAGE:
                mPeriodicIO.extenderDemand = Util.clamp(mPeriodicIO.extenderDemand, -1.0, 1.0);
                mPeriodicIO.extenderFeedforward = 0.0;
                break;
            default:
                mPeriodicIO.extenderDemand = 0.0;
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        switch (armState) {
            case HOMING:
                armMotorLeader.set(ControlMode.PercentOutput, mPeriodicIO.armDemand, DemandType.ArbitraryFeedForward,
                        mPeriodicIO.armFeedforward);
                break;
            case ANGLE:
                armMotorLeader.set(ControlMode.MotionMagic,
                        Conversions.degreesToFalcon(mPeriodicIO.armDemand, Constants.SUBSYSTEM_ARM.GEAR_RATIO),
                        DemandType.ArbitraryFeedForward,
                        mPeriodicIO.armFeedforward);
                break;
            case PERCENTAGE:
                armMotorLeader.set(ControlMode.PercentOutput, mPeriodicIO.armDemand, DemandType.ArbitraryFeedForward,
                        mPeriodicIO.armFeedforward);
                break;
            default:
                armMotorLeader.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
                break;

        }
        armMotorFollower.set(ControlMode.Follower, Constants.CANID.ARM_MOTOR_LEADER);

        switch (extenderState) {
            case HOMING:
                extenderMotor.set(ControlMode.PercentOutput, mPeriodicIO.extenderDemand,
                        DemandType.ArbitraryFeedForward,
                        mPeriodicIO.extenderFeedforward);
                break;
            case LENGTH:
                if (!armIsHomed) {
                    extenderMotor.set(ControlMode.PercentOutput, 0.0);
                } else {
                    extenderMotor.set(ControlMode.MotionMagic,
                            Conversions.degreesToFalcon(
                                    mPeriodicIO.extenderDemand / Constants.SUBSYSTEM_EXTENDER.WHEEL_CIRCUMFERENCE
                                            * 360.0,
                                    Constants.SUBSYSTEM_EXTENDER.GEAR_RATIO),
                            DemandType.ArbitraryFeedForward,
                            mPeriodicIO.extenderFeedforward);
                }
                break;
            case PERCENTAGE:
                if (!armIsHomed) {
                    extenderMotor.set(ControlMode.PercentOutput, 0.0);
                } else {
                    extenderMotor.set(ControlMode.PercentOutput, mPeriodicIO.extenderDemand,
                            DemandType.ArbitraryFeedForward,
                            mPeriodicIO.extenderFeedforward);
                }
                break;
            default:
                extenderMotor.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
                break;
        }
    }

    @Override
    public synchronized void telemetry() {
        armMech.setAngle(getAngle());
        System.out.println(getLength());
        armMech.setLength(getLength());

        Logger.getInstance().processInputs("Arm and Extender", mPeriodicIO);
        Logger.getInstance().recordOutput("Arm Mechanism", mechDrawing);
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
    public synchronized void simulate() {
        armIsHomed = true;
        extenderIsHomed = true;
    }

    public enum ARM_STATE {
        ANGLE,
        PERCENTAGE,
        HOMING
    }

    public enum EXTENDER_STATE {
        LENGTH,
        PERCENTAGE,
        HOMING
    }

    private ARM_STATE armState = ARM_STATE.HOMING;
    private EXTENDER_STATE extenderState = EXTENDER_STATE.HOMING;
}
