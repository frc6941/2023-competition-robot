package frc.robot.utils;

public class GetNearestNumber {
    public static double apply(double value, double[] targets) {
        double smallest = Double.POSITIVE_INFINITY;
        double recorder = value;

        for(double target: targets) {
            double delta = Math.abs(target - value);
            if(delta < smallest) {
                smallest = delta;
                recorder = target;
            }
        }

        return recorder;
    }

    public static int apply(int value, int[] targets) {
        double smallest = Double.POSITIVE_INFINITY;
        int recorder = value;

        for(int target: targets) {
            double delta = Math.abs(target - value);
            if(delta < smallest) {
                smallest = delta;
                recorder = target;
            }
        }

        return recorder;
    }

    public static int getIndex(int value, int[] targets) {
        double smallest = Double.POSITIVE_INFINITY;
        int recorder = value;

        for(int i = 0; i < targets.length; i++) {
            double delta = Math.abs(targets[i] - value);
            if(delta < smallest) {
                smallest = delta;
                recorder = i;
            }
        }

        return recorder;
    }
}
