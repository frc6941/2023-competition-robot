package frc.robot.commands;

import java.util.function.Supplier;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveTeleopCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSupersturcture;
    Supplier<Translation2d> translationSupplier;
    Supplier<Double> rotationSupplier;
    Supplier<Boolean> reducdMode1Activate;
    Supplier<Boolean> reducdMode2Activate;
    Supplier<Double> extesionPercentageSupplier;
    Supplier<Boolean> speedLimitActivate;
    boolean isOpenLoop;

    private ChassisSpeeds previousVelocity;
    private double previousLinearMagnitude;

    private LoggedTunableNumber turnAidControllerKp = new LoggedTunableNumber("Turn Aid Controller KP", 0.1);
    private PIDController turnAidController = new PIDController(turnAidControllerKp.get(), 0.0, 0.0);
    private Double lockAngleRecord = null;

    private static final LoggedTunableNumber maxExtensionVelocity = new LoggedTunableNumber("Max Extension Velocity", 2.5);
    private static final LoggedTunableNumber minExtensionLinearAcceleration = new LoggedTunableNumber("Min Extension Linear Acceleration", 15.0);
    private static final LoggedTunableNumber maxExtensionLinearAcceleration = new LoggedTunableNumber("Max Extension Linear Acceleration", 2.0);
    private static final LoggedTunableNumber minExtensionThetaAcceleration = new LoggedTunableNumber("Min Extension Theta Acceleration", 300.0);
    private static final LoggedTunableNumber maxExtensionThetaAcceleration = new LoggedTunableNumber("Max Extension Theta Acceleration", 150.0);

    public DriveTeleopCommand(
            SJTUSwerveMK5Drivebase mDrivebase,
            ArmAndExtender mSuperstructure,
            Supplier<Translation2d> translationSupplier,
            Supplier<Double> rotationSupplier,
            Supplier<Boolean> reducdMode1Activate,
            Supplier<Boolean> reducdMode2Activate,
            Supplier<Boolean> speedLimitActivate,
            Supplier<Double> extensionPercentageSupplier,
            boolean isOpenLoop) {
        this.mDrivebase = mDrivebase;
        this.mSupersturcture = mSuperstructure;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.reducdMode1Activate = reducdMode1Activate;
        this.reducdMode2Activate = reducdMode2Activate;
        this.speedLimitActivate = speedLimitActivate;
        this.extesionPercentageSupplier = extensionPercentageSupplier;
        this.isOpenLoop = isOpenLoop;
        addRequirements(mDrivebase);
        turnAidController.enableContinuousInput(0, 360.0);
    }

    @Override
    public void initialize() {
        mDrivebase.unbrake();
        previousVelocity = new ChassisSpeeds();
        previousLinearMagnitude = 0.0;
        lockAngleRecord = null;
    }

    @Override
    public void execute() {
        if(turnAidControllerKp.hasChanged()) {
            System.out.println("Turn Aid Controller KP Changed!");
            turnAidController.setP(turnAidControllerKp.get());
        }

        Translation2d translation = translationSupplier.get();
        Rotation2d linearDirection = new Rotation2d(translation.getX(), translation.getY());
        double rotation = rotationSupplier.get();

        double linearMagnitude = MathUtil.applyDeadband(translation.getNorm(), 0.07);
        double rotationalVelocity = MathUtil.applyDeadband(rotation, 0.07);

        if(reducdMode1Activate.get()) {
            rotationalVelocity *= 0.35;
        }

        if(reducdMode2Activate.get()) {
            rotationalVelocity *= 0.2;
            linearMagnitude *= 0.5;
        }
        
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(
            new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d())
        ).getTranslation();
        double linearValue = MathUtil.interpolate(Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_LINEAR_VELOCITY, speedLimitActivate.get() ? maxExtensionVelocity.get() : Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_LINEAR_VELOCITY, extesionPercentageSupplier.get());

        rotationalVelocity = Units.degreesToRadians(rotationalVelocity * Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_ANGULAR_VELOCITY);
        boolean shouldLockAngle = Math.abs(mDrivebase.getLocalizer().getSmoothedVelocity().getRotation().getDegrees()) < 10.0 && rotationalVelocity == 0.0;
        if(shouldLockAngle) {
            if(lockAngleRecord == null) {
                lockAngleRecord = mDrivebase.getLocalizer().getLatestPose().getRotation().getDegrees();
            }
            rotationalVelocity = turnAidController.calculate(mDrivebase.getLocalizer().getLatestPose().getRotation().getDegrees(), lockAngleRecord);
        } else {
            lockAngleRecord = null;
        }

        ChassisSpeeds desiredVelocity = new ChassisSpeeds(
            linearVelocity.getX() * linearValue,
            linearVelocity.getY() * linearValue,
            rotationalVelocity
        );

        double maxLinearAcceleration = MathUtil.interpolate(minExtensionLinearAcceleration.get(), maxExtensionLinearAcceleration.get(), extesionPercentageSupplier.get());
        double maxRotationalAccelearation = MathUtil.interpolate(minExtensionThetaAcceleration.get(), maxExtensionThetaAcceleration.get(), extesionPercentageSupplier.get());

        boolean accLimit;
        if(previousLinearMagnitude > linearMagnitude) {
            accLimit = true;
        } else {
            accLimit = false;
        }
        previousLinearMagnitude = linearMagnitude;

        if(accLimit){
            desiredVelocity = new ChassisSpeeds(
                MathUtil.clamp(
                    desiredVelocity.vxMetersPerSecond,
                    previousVelocity.vxMetersPerSecond - maxLinearAcceleration * Constants.LOOPER_DT,
                    previousVelocity.vxMetersPerSecond + maxLinearAcceleration * Constants.LOOPER_DT
                ),
                MathUtil.clamp(
                    desiredVelocity.vyMetersPerSecond,
                    previousVelocity.vyMetersPerSecond - maxLinearAcceleration * Constants.LOOPER_DT,
                    previousVelocity.vyMetersPerSecond + maxLinearAcceleration * Constants.LOOPER_DT
                ),
                MathUtil.clamp(
                    desiredVelocity.omegaRadiansPerSecond,
                    previousVelocity.omegaRadiansPerSecond - Units.degreesToRadians(maxRotationalAccelearation) * Constants.LOOPER_DT,
                    previousVelocity.omegaRadiansPerSecond + Units.degreesToRadians(maxRotationalAccelearation) * Constants.LOOPER_DT
                )
            );
        }
        previousVelocity = desiredVelocity;

        mDrivebase.drive(new Translation2d(desiredVelocity.vxMetersPerSecond, desiredVelocity.vyMetersPerSecond), rotationalVelocity, true, isOpenLoop, true);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
