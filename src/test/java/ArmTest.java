import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.ArmAndExtender;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class ArmTest {
    static final double DELTA = 0.001;

    ArmAndExtender armAndExtender;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        armAndExtender = ArmAndExtender.getInstance();
    }

    @AfterEach
    void shutdown() throws Exception {

    }
}
