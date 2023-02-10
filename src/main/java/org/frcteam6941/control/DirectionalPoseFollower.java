package org.frcteam6941.control;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DirectionalPoseFollower {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;

    private DirectionalPose2d targetPose = null;
    private boolean needReset = false;

    public DirectionalPoseFollower(ProfiledPIDController xController, ProfiledPIDController yController,
            ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }

    public void setTargetPose(DirectionalPose2d targetPose) {
        this.targetPose = targetPose;
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = new DirectionalPose2d(targetPose, true, true, true);
    }

    public DirectionalPose2d getTargetPose() {
        return targetPose;
    }

    public void clear() {
        targetPose = null;
        needReset = true;
    }

    public Optional<HolonomicDriveSignal> update(Pose2d currentPose, Pose2d currentVelocity, HolonomicDriveSignal inputDriveSignal) {
        if (targetPose == null) {
            needReset = true;
            return Optional.empty();
        } else {
            if(needReset) {
                xController.reset(currentPose.getX(), currentVelocity.getX());
                yController.reset(currentPose.getY(), currentVelocity.getY());
                thetaController.reset(currentPose.getRotation().getDegrees(),currentVelocity.getRotation().getDegrees());
                needReset = false;
            }
            double x = inputDriveSignal.getTranslation().getX();
            double y = inputDriveSignal.getTranslation().getY();
            double theta = inputDriveSignal.getRotation();
            if (targetPose.isXRestricted()) {
                x = xController.calculate(currentPose.getX(), targetPose.getX());
            }
            if (targetPose.isYRestricted()) {
                y = yController.calculate(currentPose.getY(), targetPose.getY());
            }
            if (targetPose.isThetaRestricted()) {
                theta = thetaController.calculate(currentPose.getRotation().getDegrees(),
                        targetPose.getRotation().getDegrees());
            }
            return Optional.of(
                    new HolonomicDriveSignal(
                            new Translation2d(x, y),
                            theta,
                            true,
                            false));
        }
    }
    
    public boolean isXRestricted() {
        return (targetPose == null ? false : targetPose.isXRestricted());
    }

    public boolean isYRestricted() {
        return (targetPose == null ? false : targetPose.isYRestricted());
    }

    public boolean isThetaRestricted() {
        return (targetPose == null ? false : targetPose.isThetaRestricted());
    }
}
