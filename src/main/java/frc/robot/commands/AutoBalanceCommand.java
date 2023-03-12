package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class AutoBalanceCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    private static final LoggedTunableNumber autoBalanceAdjustmentSpeed = new LoggedTunableNumber(
            "Auto Balance Adjustment Speed", 0.15);
    private static final LoggedTunableNumber autoBalanceClimbSpeed = new LoggedTunableNumber("Auto Balance Climb Speed",
            0.4);
    private static final LoggedTunableNumber autoBalanceDashSpeed = new LoggedTunableNumber("Auto Balance Dash Speed",
            0.8);
    private static final LoggedTunableNumber autoOnPlatformDegreeThreshold = new LoggedTunableNumber(
            "On Platform Angle Degree Threshold", 8.0);

    private static final LoggedTunableNumber autoBalancePositionDegreeThreshold = new LoggedTunableNumber(
            "Auto Balance Angle Degree Threshold", 10.0);

    private static final LoggedTunableNumber autoBalanceAngularVelocityThreshold = new LoggedTunableNumber(
            "Auto Balance Angular Velocity DegSec Threshold", 2.0);

    private DoubleSupplier direction;

    private boolean isOnPlatform;
    private boolean isInitialPositive;
    private boolean hasOvershooted;

    public AutoBalanceCommand(SJTUSwerveMK5Drivebase mDrivebase, DoubleSupplier direction) {
        this.mDrivebase = mDrivebase;
        this.direction = direction;
    }

    public void initialize() {
        this.mDrivebase.unbrake();
    }

    public void execute() {
        Rotation2d drivebaseDirection = mDrivebase.getLocalizer().getLatestPose().getRotation();
        double roll = mDrivebase.getRoll();
        double pitch = mDrivebase.getPitch();
        double rollV = mDrivebase.getRollVelocity();
        double pitchV = mDrivebase.getPtichVelocity();

        double angle = drivebaseDirection.getCos() * pitch + drivebaseDirection.getSin() * roll;
        double angularVelocity = drivebaseDirection.getCos() * pitchV + drivebaseDirection.getSin() * rollV;

        boolean stationTipping = (angle < 0.0 && angularVelocity > autoBalanceAngularVelocityThreshold.get())
                || (angle > 0.0 && angularVelocity < autoBalanceAngularVelocityThreshold.get());
        boolean stationLevel = Math.abs(angle) < autoBalancePositionDegreeThreshold.get();

        if (Math.abs(angle) > autoOnPlatformDegreeThreshold.get() && !isOnPlatform) {
            isOnPlatform = true;
            isInitialPositive = angle > 0.0;
        }

        if (isOnPlatform) {
            if ((isInitialPositive && angle < -autoOnPlatformDegreeThreshold.get())
                    || (!isInitialPositive && angle > autoOnPlatformDegreeThreshold.get())) {
                hasOvershooted = true;
            }

            if(hasOvershooted) {
                if (stationTipping && stationLevel) {
                    mDrivebase.brake();
                } else {
                    mDrivebase.unbrake();
                    mDrivebase.drive(new Translation2d(autoBalanceAdjustmentSpeed.get() * (angle > 0.0 ? 1.0 : -1.0), 0.0), 0.0, true,
                        false);
                    mDrivebase.setHeadingTarget(direction.getAsDouble());
                    mDrivebase.setLockHeading(true);
                }
            } else {
                mDrivebase.unbrake();
                mDrivebase.drive(new Translation2d(autoBalanceClimbSpeed.get() * (angle > 0.0 ? 1.0 : -1.0), 0.0), 0.0, true,
                        false);
                mDrivebase.setHeadingTarget(direction.getAsDouble());
                mDrivebase.setLockHeading(true);
            }
        } else {
            mDrivebase.drive(new Translation2d(autoBalanceDashSpeed.get(), 0.0), 0.0, true, false);
            mDrivebase.setHeadingTarget(direction.getAsDouble());
            mDrivebase.setLockHeading(true);
        }

        SmartDashboard.putNumber("Auto Balance Angle", angle);
        SmartDashboard.putNumber("Auto Balance Velocity DegSec2", angularVelocity);
        SmartDashboard.putBoolean("Is On Platform", isOnPlatform);
        SmartDashboard.putBoolean("Has Overshooted", hasOvershooted);
    }

    public void end(boolean interrupted) {
        mDrivebase.unbrake();
        isOnPlatform = false;
        mDrivebase.setLockHeading(false);
    }

    public boolean isFinished() {
        return false;
    }
}
