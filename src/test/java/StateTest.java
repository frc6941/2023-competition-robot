import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.states.SuperstructureState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class StateTest {
    static final double DELTA = 0.001;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void testArmOnTarget() {
        var armThreshold = 1;
        var desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(4),
                100
        );
        var currentState = new SuperstructureState(
                Rotation2d.fromDegrees(3.3),
                10
        );
        assertTrue(currentState.isArmOnTarget(desiredState, armThreshold));
        assertFalse(currentState.isExtenderOnTarget(desiredState, 1));
    }

    @Test
    void testExtenderOnTarget() {
        var extenderThreshold = 1;
        var desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(100),
                4
        );
        var currentState = new SuperstructureState(
                Rotation2d.fromDegrees(4),
                3.3
        );
        assertTrue(currentState.isExtenderOnTarget(desiredState, extenderThreshold));
        assertFalse(currentState.isArmOnTarget(desiredState, 1));
    }

    @Test
    void testOnTarget() {
        var armThreshold = 1;
        var extenderThreshold = 1;
        var desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(4),
                4
        );
        var currentState = new SuperstructureState(
                Rotation2d.fromDegrees(3.3),
                3.3
        );
        assertTrue(currentState.isOnTarget(desiredState, armThreshold, extenderThreshold));
    }
}
