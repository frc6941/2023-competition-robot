package frc.robot.coordinators;

import java.util.ArrayList;
import java.util.Optional;

import org.frcteam6941.localization.Localizer;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.vision.CameraConstants;
import org.frcteam6941.vision.EstimatedPoseProvider;
import org.frcteam6941.vision.EstimatedPoseProvider.EstimatedPoseWithDistance;
import org.frcteam6941.vision.PhotonCameraEstimatedPoseProvider;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.PolynomialRegression;

public class RobotStateEstimator implements Updatable {
    private Localizer localizer = SJTUSwerveMK5Drivebase.getInstance().getLocalizer();
    private SJTUSwerveMK5Drivebase drivebase = SJTUSwerveMK5Drivebase.getInstance();
    private ArrayList<EstimatedPoseProvider> providers = new ArrayList<>();

    private PolynomialRegression xyStdDevModel =
            new PolynomialRegression(
                new double[] {
                  0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                  3.223358, 4.093358, 4.726358
                },
                new double[] {
                  0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.15, 0.20, 0.25, 0.30
                },
                1);

    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    private RobotStateEstimator() {
        for (CameraConstants camera : Constants.SUBSYSTEM_VISION.CAMERA_CONSTANTS) {
            providers.add(new PhotonCameraEstimatedPoseProvider(camera, FieldConstants.TEST_LAYOUT));
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
                Optional<EstimatedPoseWithDistance> ePose = poseProvider.getEstimatedPose(localizer.getLatestPose());
                if(ePose.isPresent()) {
                    EstimatedPoseWithDistance eposeWithDistance = ePose.get();
                    Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " Estimate",
                            eposeWithDistance.pose.estimatedPose.toPose2d());
                    double xyStdDev = xyStdDevModel.predict(eposeWithDistance.distance);
                    localizer.addMeasurement(eposeWithDistance.pose.timestampSeconds,
                        new Pose2d(eposeWithDistance.pose.estimatedPose.toPose2d().getTranslation(), Rotation2d.fromDegrees(drivebase.getYaw())),
                            new Pose2d(xyStdDev, xyStdDev, Rotation2d.fromDegrees(2.0))
                        );
                }


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
