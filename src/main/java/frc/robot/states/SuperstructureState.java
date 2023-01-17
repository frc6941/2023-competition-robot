package frc.robot.states;

import com.team254.lib.util.Util;

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
    // The angle of the large base arm.
    public double armAngle = 0.0;
    // The length of the extender installed on the base arm.
    public double extenderLength = 10.0;

    // If is openloop.
    public boolean isOpenLoop = false;
    // If arm is in openloop, the corresponding control power from -1 to 1.
    public double armOpenLoopPower = 0.0;
    // If extender is in openloop, the corresponding control power from -1 to 1.
    public double extenderOpenLoopPower = 0.0;

    public SuperstructureState(
            double armAngle, double extenderLength,
            boolean isOpenLoop, double armOpenLoopPower, double extenderOpenLoopPower) {
        this.armAngle = armAngle;
        this.extenderLength = extenderLength;

        this.isOpenLoop = isOpenLoop;
        this.armOpenLoopPower = armOpenLoopPower;
        this.extenderOpenLoopPower = extenderOpenLoopPower;
    }

    public SuperstructureState(double armAngle, double extenderLength) {
        this(armAngle, extenderLength, false, 0.0, 0.0);
    }

    public SuperstructureState() {

    }

    public boolean hasChangedScoringGoal(SuperstructureState other) {
        return other.armAngle == armAngle && other.extenderLength == this.extenderLength;
    }

    public boolean isOnTarget(SuperstructureState desiredState, double armThreshold, double extenderThreshold) {
        return Util.epsilonEquals(desiredState.armAngle, armAngle, armThreshold)
                && Util.epsilonEquals(desiredState.extenderLength, extenderLength, extenderThreshold);
    }

    public SuperstructureState getInRange(double armMax, double armMin, double extenderMax, double extenderMin) {
        armAngle = Util.clamp(armAngle, armMin, armMax);
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
        return c.armAngle == armAngle && c.extenderLength == extenderLength;
    }
}
