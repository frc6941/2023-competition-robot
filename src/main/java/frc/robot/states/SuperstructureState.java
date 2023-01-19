package frc.robot.states;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.SuperstructureConstants.ARM_ANGLES;
import frc.robot.SuperstructureConstants.EXTENDER_LENGTHS;

/*
 * Representing the state of superstructure.
 * 
 * Positive direction definition:
 * The tip of the "triangle" form by the supporting rod and the drive train is the "head" of the drivetrain.
 * 
 */
public class SuperstructureState {
    // The angle of the large base arm - viewed from the right, CCW positive.
    public Rotation2d armAngle = new Rotation2d();
    // The length of the extender installed on the base arm.
    public double extenderLength = 10.0;

    public SuperstructureState(Rotation2d armAngle, double extenderLength) {
        this.armAngle = armAngle;
        this.extenderLength = extenderLength;
    }

    public SuperstructureState() {

    }

    public boolean hasChangedScoringGoal(SuperstructureState other) {
        return other.armAngle == armAngle && other.extenderLength == this.extenderLength;
    }

    public boolean isOnTarget(SuperstructureState desiredState, double armThreshold, double extenderThreshold) {
        return Util.epsilonEquals(desiredState.armAngle.getDegrees(), armAngle.getDegrees(), armThreshold)
                && Util.epsilonEquals(desiredState.extenderLength, extenderLength, extenderThreshold);
    }

    public SuperstructureState getInRange(double armMax, double armMin, double extenderMax, double extenderMin) {
        armAngle = Rotation2d.fromDegrees(Util.clamp(armAngle.getDegrees(), armMin, armMax));
        extenderLength = Util.clamp(extenderLength, extenderMin, extenderMax);
        return this;
    }

    public SuperstructureState getInRange() {
        return this.getInRange(
            ARM_ANGLES.MAX, ARM_ANGLES.MIN,
            EXTENDER_LENGTHS.MAX, EXTENDER_LENGTHS.MIN);
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) {
            return true;
        }
 
        if (!(o instanceof SuperstructureState)) {
            return false;
        }
         
        SuperstructureState c = (SuperstructureState) o;
        return c.armAngle.equals(c.armAngle) && c.extenderLength == extenderLength;
    }

    @Override
    public String toString() {
        return String.format("Superstructure State: Arm: %.2f deg; Extender: %.2f m", armAngle.getDegrees(), extenderLength);
    }
}
