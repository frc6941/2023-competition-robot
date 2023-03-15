package frc.robot.commands;

import org.frcteam6328.utils.LoggedTunableNumber;

import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class AutoBalanceCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    private static final LoggedTunableNumber autoBalanceAdjustmentTime = new LoggedTunableNumber(
            "Auto Balance Back Adjustment Time", 0.05);
    private static final LoggedTunableNumber autoBalanceClimbSpeed = new LoggedTunableNumber("Auto Balance Climb Speed",
            0.4);
    private static final LoggedTunableNumber autoOnPlatformDegreeThreshold = new LoggedTunableNumber(
            "On Platform Angle Degree Threshold", 8.0);
    private static final LoggedTunableNumber autoBalancePositionDegreeThreshold = new LoggedTunableNumber(
            "Auto Balance Angle Degree Threshold", 6.0);
    private static final LoggedTunableNumber autoBalanceAngularVelocityThreshold = new LoggedTunableNumber(
            "Auto Balance Angular Velocity DegSec Threshold", 3.0);

    private TimeDelayedBoolean brake = new TimeDelayedBoolean();

    private boolean isOnPlatform;
    private boolean isInitialPositive;
    private boolean hasOvershooted;

    public AutoBalanceCommand(SJTUSwerveMK5Drivebase mDrivebase) {
        this.mDrivebase = mDrivebase;
        this.isOnPlatform = false;
        this.isInitialPositive = false;
        this.hasOvershooted = false;
    }

    @Override
    public void initialize() {
        this.mDrivebase.unbrake();
        this.isOnPlatform = false;
        this.isInitialPositive = false;
        this.hasOvershooted = false;
        brake.update(false, 0.0);
    }

    @Override
    public void execute() {
        Rotation2d drivebaseDirection = mDrivebase.getLocalizer().getLatestPose().getRotation();
        double roll = mDrivebase.getRoll();
        double pitch = mDrivebase.getPitch();
        double rollV = mDrivebase.getRollVelocity();
        double pitchV = mDrivebase.getPitchVelocity();

        double angle = drivebaseDirection.getCos() * pitch + drivebaseDirection.getSin() * roll;
        double angularVelocity = drivebaseDirection.getCos() * pitchV + drivebaseDirection.getSin() * rollV;

        boolean stationTipping = (angle < 0.0 && angularVelocity > autoBalanceAngularVelocityThreshold.get())
                || (angle > 0.0 && angularVelocity < autoBalanceAngularVelocityThreshold.get());
        boolean stationLevel = Math.abs(angle) < autoBalancePositionDegreeThreshold.get();

        // if (Math.abs(angle) > autoOnPlatformDegreeThreshold.get() && !isOnPlatform) {
        //     isOnPlatform = true;
        //     isInitialPositive = angle > 0.0;
        // }

        // if (isOnPlatform) {
        //     if ((isInitialPositive && angle < -autoOnPlatformDegreeThreshold.get())
        //             || (!isInitialPositive && angle > autoOnPlatformDegreeThreshold.get())) {
        //         hasOvershooted = true;
        //     }

        //     if(hasOvershooted) {
        //         if (brake.update(true, autoBalanceAdjustmentTime.get())) {
        //             mDrivebase.brake();
        //         } else {
        //             mDrivebase.unbrake();
        //             mDrivebase.drive(new Translation2d(autoBalanceClimbSpeed.get() * (isInitialPositive ? - 1.0 : 1.0), 0.0), 0.0, true,
        //                 false, true);
        //         }
        //     } else {
        //         mDrivebase.unbrake();
        //         mDrivebase.drive(new Translation2d(autoBalanceClimbSpeed.get() * (angle > 0.0 ? 1.0 : -1.0), 0.0), 0.0, true,
        //                 false, true);
        //     }

            
        // }

        if(stationLevel && stationTipping) {
            mDrivebase.brake();
        } else {
            mDrivebase.drive(new Translation2d(autoBalanceClimbSpeed.get() * (angle > 0.0 ? 1.0 : -1.0), 0.0), 0.0, true, false, true);
        }

        SmartDashboard.putNumber("Auto Balance Angle", angle);
        SmartDashboard.putNumber("Auto Balance Velocity DegSec2", angularVelocity);
        SmartDashboard.putBoolean("Is On Platform", isOnPlatform);
        SmartDashboard.putBoolean("Has Overshooted", hasOvershooted);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.unbrake();
        mDrivebase.setLockHeading(false);
        this.isOnPlatform = false;
        this.isInitialPositive = false;
        this.hasOvershooted = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
