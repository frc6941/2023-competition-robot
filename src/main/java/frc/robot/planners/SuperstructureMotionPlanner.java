package frc.robot.planners;

import java.util.LinkedList;

import frc.robot.SuperstructureConstants.ARM_ANGLES;
import frc.robot.SuperstructureConstants.EXTENDER_LENGTHS;
import frc.robot.SuperstructureConstants.THRESHOLD;
import frc.robot.states.SuperstructureState;

public class SuperstructureMotionPlanner {
    class Action {
        public SuperstructureState desiredState;

        public Action(SuperstructureState desiredState) {
            this.desiredState = desiredState;
        }

        public boolean isFinished(SuperstructureState currentState) {
            return currentState.isOnTarget(desiredState, THRESHOLD.ARM, THRESHOLD.EXTENDER);
        }
    }

    class WaitForGoal extends Action {
        public WaitForGoal(SuperstructureState desiredState) {
            super(desiredState);
        }
    }

    class WaitForArm extends Action {
        public WaitForArm(SuperstructureState desiredState) {
            super(desiredState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return currentState.isOnTarget(desiredState, THRESHOLD.ARM, Double.POSITIVE_INFINITY);
        }
    }

    private LinkedList<Action> actionList;
    private Action currentAction = null;

    public SuperstructureMotionPlanner() {

    }
    
    public void setDesiredState(SuperstructureState desiredStateIn, SuperstructureState currentState) {
        SuperstructureState desiredState = desiredStateIn.getInRange();

        // Overhead is defined as:
        // The robot arm tries to go from one direction to another;
        boolean overhead = 
            (desiredState.armAngle < ARM_ANGLES.HEAD_ZONE_MIN_ANGLE && currentState.armAngle > ARM_ANGLES.HEAD_ZONE_MAX_ANGLE)
            || (desiredState.armAngle > ARM_ANGLES.HEAD_ZONE_MAX_ANGLE && currentState.armAngle < ARM_ANGLES.HEAD_ZONE_MIN_ANGLE);

        // TODO: add passing over danger point check when raising up.

        // Start adding actions to the motion planner.
        // First, clear all the actions in the list.
        currentAction = null;
        actionList.clear();

        // Then, add actions according to different judging conditions.
        if (overhead) {
            SuperstructureState changedState = desiredState;
            if (desiredState.armAngle < ARM_ANGLES.HEAD_ZONE_MIN_ANGLE) {
                changedState.armAngle = ARM_ANGLES.HEAD_ZONE_MIN_ANGLE;
                changedState.extenderLength = EXTENDER_LENGTHS.MIN;
            } else {
                changedState.armAngle = ARM_ANGLES.HEAD_ZONE_MAX_ANGLE;
                changedState.extenderLength = EXTENDER_LENGTHS.MIN;
            }
            actionList.add(new WaitForArm(changedState));
            actionList.add(new WaitForGoal(desiredState));
        } else {
            actionList.add(new WaitForGoal(desiredState));
        }
    }

    public SuperstructureState updateMotionPlanner(SuperstructureState currentState) {
        
        if (currentAction == null && !actionList.isEmpty()) {
            currentAction = actionList.remove();
        }

        SuperstructureState returningState;

        if (currentAction != null) {
            returningState = currentAction.desiredState.getInRange();
            if (currentAction.isFinished(currentState) && !actionList.isEmpty()) {
                currentAction = null;
            }
        } else {
            returningState = currentState;
        }
        
        return returningState;
    }

    public boolean isFinished(SuperstructureState currentState) {
        return currentAction != null && actionList.isEmpty() && currentAction.isFinished(currentState);
    }
}
