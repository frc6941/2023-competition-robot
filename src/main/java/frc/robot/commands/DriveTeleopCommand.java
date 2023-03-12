package frc.robot.commands;

import java.util.function.Supplier;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
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
    Supplier<Boolean> reducdModeActivate;
    Supplier<Double> extesionPercentageSupplier;
    boolean isOpenLoop;

    private ChassisSpeeds previousVelocity;

    private static final LoggedTunableNumber maxExtensionVelocity = new LoggedTunableNumber("Max Extension Velocity", 2.0);
    private static final LoggedTunableNumber minExtensionLinearAcceleration = new LoggedTunableNumber("Min Extension Linear Acceleration", 10.0);
    private static final LoggedTunableNumber maxExtensionLinearAcceleration = new LoggedTunableNumber("Max Extension Linear Acceleration", 2.5);
    private static final LoggedTunableNumber minExtensionThetaAcceleration = new LoggedTunableNumber("Min Extension Theta Acceleration", 600.0);
    private static final LoggedTunableNumber maxExtensionThetaAcceleration = new LoggedTunableNumber("Max Extension Theta Acceleration", 150.0);

    public DriveTeleopCommand(
            SJTUSwerveMK5Drivebase mDrivebase,
            ArmAndExtender mSuperstructure,
            Supplier<Translation2d> translationSupplier,
            Supplier<Double> rotationSupplier,
            Supplier<Boolean> reducdModeActivate,
            Supplier<Double> extensionPercentageSupplier,
            boolean isOpenLoop) {
        this.mDrivebase = mDrivebase;
        this.mSupersturcture = mSuperstructure;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.reducdModeActivate = reducdModeActivate;
        this.extesionPercentageSupplier = extensionPercentageSupplier;
        this.isOpenLoop = isOpenLoop;
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {
        mDrivebase.unbrake();
        previousVelocity = new ChassisSpeeds();
    }

    @Override
    public void execute() {
        Translation2d translation = translationSupplier.get();
        Rotation2d linearDirection = new Rotation2d(translation.getX(), translation.getY());
        double rotation = rotationSupplier.get();

        double linearMagnitude = MathUtil.applyDeadband(translation.getNorm(), 0.07);
        double rotationalVelocity = MathUtil.applyDeadband(rotation, 0.07);

        if(reducdModeActivate.get()) {
            linearMagnitude *= 0.5;
            rotationalVelocity *= 0.3;
        }
        

        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(
            new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d())
        ).getTranslation();
        rotationalVelocity = Units.degreesToRadians(rotationalVelocity * Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_ANGULAR_VELOCITY);
        double linearValue = MathUtil.interpolate(Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_LINEAR_VELOCITY, maxExtensionVelocity.get(), extesionPercentageSupplier.get());
        ChassisSpeeds desiredVelocity = new ChassisSpeeds(
            linearVelocity.getX() * linearValue,
            linearVelocity.getY() * linearValue,
            rotationalVelocity
        );

        double maxLinearAcceleration = MathUtil.interpolate(minExtensionLinearAcceleration.get(), maxExtensionLinearAcceleration.get(), extesionPercentageSupplier.get());
        double maxRotationalAccelearation = MathUtil.interpolate(minExtensionThetaAcceleration.get(), maxExtensionThetaAcceleration.get(), extesionPercentageSupplier.get());

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

        previousVelocity = desiredVelocity;

        mDrivebase.setLockHeading(false);
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
