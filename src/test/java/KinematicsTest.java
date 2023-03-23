import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.states.SuperstructureState;

public class KinematicsTest {
    @BeforeEach
    void setup() {

    }

    @Test
    void automaticCollisionAvoidance() {
        SuperstructureState desiredState = new SuperstructureState(Rotation2d.fromDegrees(-90), 5.0);
        SuperstructureState currentState = new SuperstructureState(Rotation2d.fromDegrees(-60), 5.0);

        assertEquals(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_NEGATIVE.max,
                Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.SUPERSTRUCTURE_LIMIT.optimize(
                        desiredState,
                        currentState).armAngle.getDegrees(),
                0.001);
        assertEquals(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min,
                Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.SUPERSTRUCTURE_LIMIT.optimize(
                        desiredState,
                        currentState).extenderLength,
                0.001);
    }
    
    @AfterEach
    void shutdown() throws Exception {

    }
}
