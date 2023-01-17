package frc.robot;

public class SuperstructureConstants {
    // Thresholds
    public static class THRESHOLD {
        public static double ARM = 2.0;
        public static double EXTENDER = 0.02;
    }

    // Arm Anngles
    public static class ARM_ANGLES {
        public static double MAX = 150.0;
        public static double MIN = -185.0;

        public static double HEAD_ZONE_MIN_ANGLE = -70.0;
        public static double HEAD_ZONE_MAX_ANGLE = 70.0;

        public static double BACK_RAISE_ZONE_MIN_ANGLE = MIN;
        public static double BACK_RAISE_ZONE_MAX_ANGLE = -140.0;
    }

    // Extender Lengths
    public static class EXTENDER_LENGTHS {
        public static double MAX = 1.20;
        public static double MIN = 0.80;
    }
}
