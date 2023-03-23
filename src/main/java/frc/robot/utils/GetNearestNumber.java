package frc.robot.utils;

public class GetNearestNumber {
    public static double apply(double value, double[] targets) {
        double smallest = Double.POSITIVE_INFINITY;
        double recorder = value;

        for(double target: targets) {
            if(Math.abs(target - value) < smallest) {
                recorder = target;
            }
        }

        return recorder;
    }
}
