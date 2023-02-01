package org.frcteam6941.control;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DirectionalPoseFollower {
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;

    private DirectionalPose2d targetPose = null;

    public DirectionalPoseFollower(PIDController xController, PIDController yController,
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
        return this.targetPose;
    }

    public void clear() {
        this.targetPose = null;
    }

    public Optional<HolonomicDriveSignal> update(Pose2d currentPose, HolonomicDriveSignal inputDriveSignal) {
        if (targetPose == null) {
            return Optional.empty();
        } else {
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
                            true));
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
