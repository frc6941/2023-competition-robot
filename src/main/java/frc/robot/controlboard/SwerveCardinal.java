package frc.robot.controlboard;

public class SwerveCardinal {
    public enum SWERVE_CARDINAL {
        NONE(0),

        FORWARDS(0),
        LEFT(90),
        RIGHT(270),
        BACKWARDS(180);

        public final double degrees;

        SWERVE_CARDINAL(double degrees) {
            this.degrees = degrees;
        }
    }
}
