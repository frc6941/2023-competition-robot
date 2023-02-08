package frc.robot.coordinators;

import java.util.ArrayList;

import org.frcteam6941.localization.Localizer;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.vision.CameraConstants;
import org.frcteam6941.vision.EstimatedPoseProvider;
import org.frcteam6941.vision.PhotonCameraEstimatedPoseProvider;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class RobotStateEstimator implements Updatable {
    private Localizer localizer = SJTUSwerveMK5Drivebase.getInstance().getLocalizer();
    private ArrayList<EstimatedPoseProvider> providers = new ArrayList<>();

    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    private RobotStateEstimator() {
        for (CameraConstants camera : Constants.SUBSYSTEM_VISION.CAMERA_CONSTANTS) {
            providers.add(new PhotonCameraEstimatedPoseProvider(camera, FieldConstants.LAYOUT));
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void update(double time, double dt) {
        for (EstimatedPoseProvider poseProvider : providers) {
            try {
                poseProvider.getEstimatedPose(localizer.getPoseAtTime(time))
                        .ifPresentOrElse(estimate -> {
                            localizer.addMeasurement(estimate.timestampSeconds,
                                    estimate.estimatedPose.toPose2d(),
                                    new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1.0)));
                            Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " Estimate",
                                    estimate.estimatedPose.toPose2d());
                        },
                        () -> Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " Estimate",
                                    new Pose2d(-1000.0, -1000.0, Rotation2d.fromDegrees(0.0))));

            } catch (Exception e) {
                Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " Estimate",
                                    new Pose2d(-1000.0, -1000.0, Rotation2d.fromDegrees(0.0)));
                throw e;
            }

        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void telemetry() {
        // Auto Generated Method
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
    public synchronized void simulate(double time, double dt) {
        // Auto Generated Method
    }
}
