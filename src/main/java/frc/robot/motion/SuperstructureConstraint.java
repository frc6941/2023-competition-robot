package frc.robot.motion;

import org.frcteam6941.utils.Range;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.states.SuperstructureState;

public class SuperstructureConstraint {
    private Range heightRange;
    private Range armRange;
    private Range extenderRange;


    public SuperstructureConstraint(Range heightRange, Range armRange, Range extenderRange) {
        this.heightRange = heightRange;
        this.armRange = armRange;
        this.extenderRange = extenderRange;
    }

    public SuperstructureState constrain(SuperstructureState state) {
        Translation2d finalPosition = SuperstructureKinematics.forwardKinematics2d(state);
        Translation2d newPosition = new Translation2d(finalPosition.getX(), this.heightRange.clamp(finalPosition.getY()));
        SuperstructureState inverseState = SuperstructureKinematics.inverseKinematics2d(newPosition);
        return new SuperstructureState(
            Rotation2d.fromDegrees(this.armRange.clamp(inverseState.armAngle.getDegrees())),
            this.extenderRange.clamp(state.extenderLength));
    }
}
