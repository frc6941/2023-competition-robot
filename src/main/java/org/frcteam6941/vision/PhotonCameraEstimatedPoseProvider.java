package org.frcteam6941.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonCameraEstimatedPoseProvider implements EstimatedPoseProvider{
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private String name;

    public PhotonCameraEstimatedPoseProvider(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraConstants.getCameraName());
        name = cameraConstants.getCameraName();
        Transform3d robotToCamera = new Pose3d().minus(cameraConstants.getCameraPoseToRobotCenter());
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    @Override
    public String getName() {
        return name;
    }
}
