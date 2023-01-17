package frc.robot.states;

public class CoordinatorStateMachine {
    public enum STATE {
        PREP_SCORING,
        SCORING,
        COMMUTING,
        LOADING,
        CLIMBING
    }

    public enum WANTED_ACTION {
        PREP_SCORE,
        SCORE,
        COMMUTE,
        LOAD,
        CLIMB
    }
}
