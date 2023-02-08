package frc.robot.motion;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.states.SuperstructureState;
import org.frcteam6941.utils.Peer;
import org.frcteam6941.utils.Range;

import java.util.List;

public class SuperstructureConstraint {
    private final Range heightRange;
    private final Range armRange;
    private final Range extenderRange;
    private final Range dangerousPositiveArmRange;
    private final Range dangerousNegativeArmRange;

    public SuperstructureConstraint(
            Range heightRange,
            Range armRange,
            Range extenderRange,
            Range dangerousPositiveArmRange,
            Range dangerousNegativeArmRange
    ) {
        this.heightRange = heightRange;
        this.armRange = armRange;
        this.extenderRange = extenderRange;
        this.dangerousPositiveArmRange = dangerousPositiveArmRange;
        this.dangerousNegativeArmRange = dangerousNegativeArmRange;
    }

    public SuperstructureState optimize(SuperstructureState desiredState, SuperstructureState currentState) {

        DangerousArmRangeStrategies strategies = new DangerousArmRangeStrategies(
                dangerousPositiveArmRange,
                dangerousNegativeArmRange
        );

        // Clamp state into max and min
        SuperstructureState clampedDesiredState = new SuperstructureState(
                Rotation2d.fromDegrees(this.armRange.clamp(desiredState.armAngle.getDegrees())),
                this.extenderRange.clamp(desiredState.extenderLength)
        );

        var strategyMatched = strategies.matchStrategy(desiredState.armAngle.getDegrees());

        if (strategyMatched.isPresent()) {
            clampedDesiredState.extenderLength = extenderRange.min;

            if (!currentState.isExtenderOnTarget(clampedDesiredState,
                    Constants.SUBSYSTEM_SUPERSTRUCTURE.THRESHOLD.EXTENDER)
            ) clampedDesiredState.armAngle = Rotation2d.fromDegrees(strategyMatched.get());

            return clampedDesiredState;
        }

        // Use forward kinematics to determine final position, then clamp height to
        // limited zone
        Translation2d finalPosition = SuperstructureKinematics.forwardKinematics2d(clampedDesiredState);
        Translation2d newPosition = new Translation2d(
                finalPosition.getX(),
                this.heightRange.clamp(finalPosition.getY())
        );

        clampedDesiredState = SuperstructureKinematics.inverseKinematics2d(newPosition);
        // clamp again to regulate the angles into arm range
        clampedDesiredState = new SuperstructureState(
                Rotation2d.fromDegrees(this.armRange.clamp(clampedDesiredState.armAngle.getDegrees())),
                this.extenderRange.clamp(clampedDesiredState.extenderLength)
        );
        return clampedDesiredState;
    }
}
