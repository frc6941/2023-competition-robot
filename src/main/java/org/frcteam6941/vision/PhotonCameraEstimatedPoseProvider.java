package org.frcteam6941.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class PhotonCameraEstimatedPoseProvider implements EstimatedPoseProvider {
    private PhotonCamera camera;
    private CameraConstants constants;
    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator poseEstimator;

    public PhotonCameraEstimatedPoseProvider(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraConstants.getCameraName());
        constants = cameraConstants;
        layout = fieldLayout;
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, camera, constants.getCameraPoseToRobotCenter().minus(new Pose3d()).inverse());
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public Optional<EstimatedPoseWithDistance> getEstimatedPose(Pose2d referenceRobotPose) {
        Optional<EstimatedRobotPose> estimate = poseEstimator.update(camera.getLatestResult());
        if(estimate.isPresent()) {
            return Optional.of(new EstimatedPoseWithDistance(
                estimate.get(),
                estimate.get().estimatedPose.getTranslation().toTranslation2d().minus(referenceRobotPose.getTranslation()).getNorm()
            ));
        } else {
            return Optional.empty();
        }
    }

    @Override
    public String getName() {
        return constants.getCameraName();
    }
}
