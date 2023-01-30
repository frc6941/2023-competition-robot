package org.frcteam6941.control;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DirectionalPoseFollower {
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    private DirectionalPose2d targetPose = null;

    public DirectionalPoseFollower(PIDController xController, PIDController yController) {
        this.xController = xController;
        this.yController = yController;
    }

    public void setTargetPose(DirectionalPose2d targetPose) {
        this.targetPose = targetPose;
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = new DirectionalPose2d(targetPose, true, true, true);
    }

    public Optional<HolonomicDriveSignal> getDriveSignal(Pose2d currentPose, HolonomicDriveSignal inputDriveSignal) {
        if (targetPose == null) {
            return Optional.empty();
        } else {
            if (currentPose.getTranslation().minus(targetPose.getTranslation()).getNorm() > 5.0) {
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
    }

    public boolean isOnTarget() {
        if (targetPose == null) {
            return false;
        } else {
            return (xController.atSetpoint() || !targetPose.isXRestricted())
                    && (yController.atSetpoint() || !targetPose.isYRestricted())
                    && (thetaController.atSetpoint() || !targetPose.isThetaRestricted());
        }
    }
}
