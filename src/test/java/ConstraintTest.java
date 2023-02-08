import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.motion.SuperstructureConstraint;
import frc.robot.states.SuperstructureState;
import org.frcteam6941.utils.Range;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ConstraintTest {
    static final double DELTA = 0.001;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    /**
     * Test when arm angle is in positive dangerous range, and extender is not on target currently.
     */
    @Test
    void testOptimize1() {
        SuperstructureConstraint superstructureConstraint = new SuperstructureConstraint(
                new Range(3, 10),
                new Range(4, 9),
                new Range(3, 10),
                new Range(3, 10),
                new Range(-10, -3)
        );
        SuperstructureState desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(3),
                10
        );
        SuperstructureState currentState = new SuperstructureState();
        SuperstructureState optimizedState = new SuperstructureState(
                Rotation2d.fromDegrees(3),
                3
        );
        assertEquals(optimizedState, superstructureConstraint.optimize(desiredState, currentState));
    }

    /**
     * Test when arm angle is in negative dangerous range, and extender is not on target currently.
     */
    @Test
    void testOptimize2() {
        SuperstructureConstraint superstructureConstraint = new SuperstructureConstraint(
                new Range(3, 10),
                new Range(-9, -4),
                new Range(3, 10),
                new Range(3, 10),
                new Range(-10, -3)
        );
        SuperstructureState desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(-5),
                10
        );
        SuperstructureState currentState = new SuperstructureState();
        SuperstructureState optimizedState = new SuperstructureState(
                Rotation2d.fromDegrees(-3),
                3
        );
        assertEquals(optimizedState, superstructureConstraint.optimize(desiredState, currentState));
    }

    /**
     * Test when arm angle is in positive dangerous range, and extender is on target currently.
     */
    @Test
    void testOptimize3() {
        SuperstructureConstraint superstructureConstraint = new SuperstructureConstraint(
                new Range(3, 10),
                new Range(4, 9),
                new Range(3, 10),
                new Range(3, 10),
                new Range(-10, -3)
        );
        SuperstructureState desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(3),
                10
        );
        SuperstructureState currentState = new SuperstructureState(
                new Rotation2d(4),
                10
        );
        SuperstructureState optimizedState = new SuperstructureState(
                Rotation2d.fromDegrees(3),
                3
        );
        assertEquals(optimizedState, superstructureConstraint.optimize(desiredState, currentState));
    }

    /**
     * Test when arm angle is in negative dangerous range, and extender is on target currently.
     */
    @Test
    void testOptimize4() {
        SuperstructureConstraint superstructureConstraint = new SuperstructureConstraint(
                new Range(3, 10),
                new Range(-9, -4),
                new Range(3, 10),
                new Range(3, 10),
                new Range(-10, -3)
        );
        SuperstructureState desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(-5),
                10
        );
        SuperstructureState currentState = new SuperstructureState(
                new Rotation2d(-4),
                3
        );
        SuperstructureState optimizedState = new SuperstructureState(
                Rotation2d.fromDegrees(-5),
                3
        );
        assertEquals(optimizedState, superstructureConstraint.optimize(desiredState, currentState));
    }

    /**
     * Test when arm angle is in neither of dangerous ranges.
     */
    @Test
    void testOptimize5() {
        SuperstructureConstraint superstructureConstraint = new SuperstructureConstraint(
                new Range(3, 10),
                new Range(-2, 2),
                new Range(3, 10),
                new Range(3, 10),
                new Range(-10, -3)
        );
        SuperstructureState desiredState = new SuperstructureState(
                Rotation2d.fromDegrees(-3),
                10
        );
        SuperstructureState currentState = new SuperstructureState();

        // TODO Don't understand the logic of Kinematics.forwardKinematics(). Can't figure out what it will return.
        SuperstructureState optimizedState = new SuperstructureState(
                Rotation2d.fromDegrees(0),
                0
        );
    }
}
