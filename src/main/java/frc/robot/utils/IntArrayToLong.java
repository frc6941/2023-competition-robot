package frc.robot.utils;

public class IntArrayToLong {
    public static long[] apply(int[] target) {
        long[] result = new long[target.length];

        for(int i = 0; i < target.length; i++) {
            result[i] = (long) target[i];
        }

        return result;
    }
}
