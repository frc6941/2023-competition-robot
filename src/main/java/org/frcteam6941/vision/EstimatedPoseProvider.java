package org.frcteam6941.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public interface EstimatedPoseProvider {
    String getName();
    Optional<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose);
}
