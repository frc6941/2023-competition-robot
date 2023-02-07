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

        List<Peer<Range, Double>> dangerousArmRangeStrategies = List.of(
            new Peer<>(dangerousPositiveArmRange, dangerousPositiveArmRange.min),
            new Peer<>(dangerousNegativeArmRange, dangerousNegativeArmRange.max)
        );

        // Clamp state into max and min
        SuperstructureState clampedDesiredState = new SuperstructureState(
                Rotation2d.fromDegrees(this.armRange.clamp(desiredState.armAngle.getDegrees())),
                this.extenderRange.clamp(desiredState.extenderLength)
        );

        // Match dangerous ranges.
        for (Peer<Range, Double> strategy : dangerousArmRangeStrategies) {
            if (!strategy.getKey().inRange(desiredState.armAngle.getDegrees())) {
                continue;
            }

            clampedDesiredState.extenderLength = extenderRange.min;
            if (!currentState.isExtenderOnTarget(clampedDesiredState,
                    Constants.SUBSYSTEM_SUPERSTRUCTURE.THRESHOLD.EXTENDER)
            ) clampedDesiredState.armAngle = Rotation2d.fromDegrees(strategy.getValue());

            return clampedDesiredState;
        }

        // If not match any dangerous range, the code will go here.
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
