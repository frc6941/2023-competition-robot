package frc.robot.states;

import frc.robot.planners.SuperstructureMotionPlanner;

public class SuperstructureStateMachine {
    public enum WANTED_ACTION {
        IDLE,
        GO_TO_POSITION,
        WANT_MANUAL
    }

    public enum STATE {
        HOLDING_POSITION,
        MOVING_TO_POSITION,
        MANUAL
    }

    private STATE systemState = STATE.HOLDING_POSITION;

    private SuperstructureState desiredState = new SuperstructureState();
    private boolean hasChangedState = true;

    private SuperstructureMotionPlanner planner = new SuperstructureMotionPlanner();

    public void setDesiredState(SuperstructureState desiredState) {
        hasChangedState = this.desiredState.equals(desiredState);
        this.desiredState = desiredState;
    }

    public synchronized SuperstructureState update(double time, WANTED_ACTION action, SuperstructureState currentState) {
        // First, handle different actions according to states
        STATE newState;

        switch (systemState) {
            case HOLDING_POSITION:
                newState = getTransition(action, currentState);
                desiredState.isOpenLoop = false;
                break;
            case MOVING_TO_POSITION:
                newState = getTransition(action, currentState);
                desiredState.isOpenLoop = false;
                break;
            case MANUAL:
                if (action != WANTED_ACTION.WANT_MANUAL) {
                    newState = getTransition(WANTED_ACTION.GO_TO_POSITION, currentState);
                    desiredState.isOpenLoop = false;
                } else {
                    newState = getTransition(action, currentState);
                    desiredState.isOpenLoop = true;
                }
                break;
            default:
                System.out.println("Unexpected State");
                newState = systemState;
                break;
        }
        
        if (newState != systemState) {
            System.out.println("State changed: from " + systemState.toString() + " to " + newState.toString());
            systemState = newState;
        }

        if (!desiredState.isOpenLoop) {
            desiredState = planner.updateMotionPlanner(currentState);
        }

        return desiredState;
    }

    public STATE getTransition(WANTED_ACTION action, SuperstructureState currentState) {
        if (action == WANTED_ACTION.GO_TO_POSITION) {
            if (hasChangedState) {
                planner.setDesiredState(desiredState, currentState);
                hasChangedState = false;
            } else if (planner.isFinished(currentState)) {
                return STATE.HOLDING_POSITION;
            }
            return STATE.MOVING_TO_POSITION;
        } else if (action == WANTED_ACTION.WANT_MANUAL) {
            return STATE.MANUAL;
        } else {
            if (systemState == STATE.MOVING_TO_POSITION && !planner.isFinished(currentState)) {
                return STATE.MOVING_TO_POSITION;
            } else {
                return STATE.HOLDING_POSITION;
            }
        }
    }

}
