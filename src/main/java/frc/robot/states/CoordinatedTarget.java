package frc.robot.states;

import edu.wpi.first.math.geometry.Pose2d;

public class CoordinatedTarget {
    public double intakerPower;
    public Pose2d robotPose;
    public SuperstructureState superstructureState;

    public CoordinatedTarget(double intakerPower, Pose2d robotPose, SuperstructureState superstructureState) {
        this.intakerPower = intakerPower;
        this.robotPose = robotPose;
        this.superstructureState = superstructureState;
    }
}
