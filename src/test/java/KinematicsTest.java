import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.SuperstructureState;

public class KinematicsTest {
    @BeforeEach
    void setup() {

    }

    @Test
    void testForwardAndInverseKinematicsMatch() {
        SuperstructureState[] testStates = new SuperstructureState[] {
                new SuperstructureState(Rotation2d.fromDegrees(-90), 5.0),
                new SuperstructureState(Rotation2d.fromDegrees(100.51), 2.46516),
                new SuperstructureState(Rotation2d.fromDegrees(-171.52), 254.9711678)
        };

        Pose2d drivetrainPose = new Pose2d(0, 0, new Rotation2d());

        for (SuperstructureState testState : testStates) {
            Translation3d endPosition = SuperstructureKinematics
                    .forwardKinematics(testState, drivetrainPose)
                    .getTranslation();
            SuperstructureState inverseState = SuperstructureKinematics.inverseKinematics(endPosition,
                    drivetrainPose);
            assertEquals(testState, inverseState);
        }
    }

    @Test
    void automaticCollisionAvoidance() {
        SuperstructureState desiredState = new SuperstructureState(Rotation2d.fromDegrees(-90), 5.0);
        SuperstructureState currentState = new SuperstructureState(Rotation2d.fromDegrees(-60), 5.0);
        SuperstructureState desiredState2 = new SuperstructureState(Rotation2d.fromDegrees(240), 5.0);
        SuperstructureState currentState2 = new SuperstructureState(Rotation2d.fromDegrees(200), 5.0);

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

        

        assertEquals(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_POSITIVE.min,
                Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.SUPERSTRUCTURE_LIMIT.optimize(
                        desiredState2,
                        currentState2).armAngle.getDegrees(),
                0.001);
        assertEquals(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min,
                Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.SUPERSTRUCTURE_LIMIT.optimize(
                        desiredState2,
                        currentState2).extenderLength,
                0.001);
    }

    @Test
    void automaticHeightClamp() {
        SuperstructureState desiredState = new SuperstructureState(Rotation2d.fromDegrees(-80), 5.0);
        SuperstructureState currentState = new SuperstructureState(Rotation2d.fromDegrees(-60), 2.0);

        assertEquals(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.HEIGHT_RANGE.min,
                SuperstructureKinematics.forwardKinematics2d(
                        Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.SUPERSTRUCTURE_LIMIT
                                .optimize(desiredState, currentState))
                        .getY(),
                0.001);
    }

    @AfterEach
    void shutdown() throws Exception {

    }
}
