package frc.robot.commands;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class AutoBalanceCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;

    private static final LoggedTunableNumber autoBalanceSpeed = new LoggedTunableNumber("Auto Balance Speed", 0.7);
    private static final LoggedTunableNumber autoBalancePositionDegreeThreshold = new LoggedTunableNumber(
            "Auto Balance Angle Degree Threshold", 5.0);
    private static final LoggedTunableNumber autoBalanceAngularVelocityThreshold = new LoggedTunableNumber(
            "Auto Balane Angular Velocity DegSec2 Threshold", 10.0);

    public AutoBalanceCommand(SJTUSwerveMK5Drivebase mDrivebase) {
        this.mDrivebase = mDrivebase;
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


        boolean stationTipping = (angle < 0.0 && angularVelocity > autoBalanceAngularVelocityThreshold.get()) || (angle > 0.0 && angularVelocity < autoBalanceAngularVelocityThreshold.get());
        boolean stationLevel = Math.abs(angle) < autoBalancePositionDegreeThreshold.get();

        if(stationTipping && stationLevel) {
            mDrivebase.brake();
        } else {
            mDrivebase.drive(new Translation2d(autoBalanceSpeed.get() * (angle > 0.0 ? -1.0 : 1.0), 0.0), 0.0, true, false);
        }

        SmartDashboard.putNumber("Auto Balance Angle", angle);
        SmartDashboard.putNumber("Auto Balance Velocity DegSec2", angularVelocity);
    }

    public void end(boolean interrupted) {
        mDrivebase.unbrake();
    }

    public boolean isFinished() {
        return false;
    }
}
