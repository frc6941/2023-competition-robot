package frc.robot.coordinators;

import java.util.Optional;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;

public class Superstructure implements Updatable {
    public static class PeriodicIO {
        /** INPUTS */
        // Swerve Variables
        public Translation2d inSwerveTranslation = new Translation2d();
        public double inSwerveRotation = 0.0;
        public SWERVE_CARDINAL inSwerveSnapRotation = SWERVE_CARDINAL.NONE;
        public boolean inSwerveBrake = false;
        public double inSwerveFieldHeadingAngle = 0.0;
        public double inSwerveAngularVelocity = 0.0;

        /** OUTPUTS */
        // Swerve Variables
        public Translation2d outSwerveTranslation = new Translation2d();
        public double outSwerveRotation = 0.0;
        public boolean outSwerveLockHeading = false;
        public double outSwerveHeadingTarget = 0.0;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();

    // Swerve setting related variables
    private boolean swerveSelfLocking = false;
    private Double swerveSelfLockheadingRecord;

    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public synchronized void updateDriverAndOperatorCommand() {
        mPeriodicIO.inSwerveTranslation = mControlBoard.getSwerveTranslation();
        mPeriodicIO.inSwerveRotation = mControlBoard.getSwerveRotation();
        mPeriodicIO.inSwerveSnapRotation = mControlBoard.getSwerveSnapRotation();
        mPeriodicIO.inSwerveBrake = mControlBoard.getSwerveBrake();

        if (mControlBoard.zeroGyro()) {
            mSwerve.resetYaw(0.0);
            Pose2d currentPose = mSwerve.getPose();
            mSwerve.getLocalizer().reset(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
            swerveSelfLockheadingRecord = null;
            mSwerve.resetHeadingController();
        }
    }


    /**
     * Update Swerve status.
     * 
     * This method should be called in the end when all the other things are
     * decided.
     */
    public synchronized void updateSwerve() {
        mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
        mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
        if (mPeriodicIO.inSwerveSnapRotation != SWERVE_CARDINAL.NONE) {
            mPeriodicIO.outSwerveLockHeading = true;
            mPeriodicIO.outSwerveHeadingTarget = mPeriodicIO.inSwerveSnapRotation.degrees;
            swerveSelfLockheadingRecord = null;
        } else if (Math.abs(mPeriodicIO.outSwerveRotation) <= 0.03
                && Math.abs(mPeriodicIO.inSwerveAngularVelocity) < 20.0
                && swerveSelfLocking) {
            mPeriodicIO.outSwerveLockHeading = true;
            swerveSelfLockheadingRecord = Optional
                    .ofNullable(swerveSelfLockheadingRecord)
                    .orElse(mPeriodicIO.inSwerveFieldHeadingAngle);
            mPeriodicIO.outSwerveHeadingTarget = swerveSelfLockheadingRecord;
        } else {
            mPeriodicIO.outSwerveLockHeading = false;
            mPeriodicIO.outSwerveHeadingTarget = 0.0;
            swerveSelfLockheadingRecord = null;
        }   
    }


    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveAngularVelocity = mSwerve.getLocalizer().getMeasuredVelocity().getRotation().getDegrees();
    }
    
    @Override
    public synchronized void update(double time, double dt){
        updateSwerve();
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Swerve Drive Logic
        if (mPeriodicIO.inSwerveBrake) {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
        } else {
            if (mPeriodicIO.inSwerveTranslation.getNorm() > Constants.CONTROLBOARD.CONTROLLER_DEADBAND
                    || mPeriodicIO.inSwerveRotation > Constants.CONTROLBOARD.CONTROLLER_DEADBAND) {
                mSwerve.getFollower().cancel();
                mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.DRIVE);
            }
            
            mSwerve.setLockHeading(mPeriodicIO.outSwerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.outSwerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, true, false);
        }

    }
    
    @Override
    public synchronized void telemetry(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void start(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        // Auto Generated Method
    }
}
