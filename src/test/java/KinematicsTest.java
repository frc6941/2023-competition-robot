import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.SuperstructureState;

public class KinematicsTest {
    @BeforeEach
    void setup() {

    }

    @Test
    void testForwardAndInverseKinematicsMatch() {
        SuperstructureState[] testStatesArray = new SuperstructureState[] {
            new SuperstructureState(Rotation2d.fromDegrees(-90), 5.0),
            new SuperstructureState(Rotation2d.fromDegrees(100.51), 2.46516),
            new SuperstructureState(Rotation2d.fromDegrees(-171.52), 254.9711678)
        };

        Pose2d drivetrainPose = new Pose2d(0,0,new Rotation2d());

        for(SuperstructureState testState: testStatesArray) {
            Translation3d endPosition = SuperstructureKinematics.forwardKinematics(testState, drivetrainPose).getTranslation();
            SuperstructureState inverseState = SuperstructureKinematics.inverseKinematics(endPosition, drivetrainPose);
            assertEquals(testState, inverseState);
        }
    }

    @AfterEach
    void shutdown() throws Exception {

    }
}
