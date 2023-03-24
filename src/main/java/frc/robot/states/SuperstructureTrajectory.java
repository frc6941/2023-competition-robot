package frc.robot.states;

import java.util.List;

import edu.wpi.first.math.interpolation.Interpolatable;

public class SuperstructureTrajectory {
    private double totalTime;
    private List<SuperstructureTrajectoryPoint> controlPoints;

    public SuperstructureTrajectory(List<SuperstructureTrajectoryPoint> controlPoints) {
        this.controlPoints = controlPoints;
    }

    public SuperstructureTrajectoryPoint sample(double timeSeconds) {
        if (timeSeconds <= controlPoints.get(0).time) {
            return controlPoints.get(0);
        }
        if (timeSeconds >= totalTime) {
            return controlPoints.get(controlPoints.size() - 1);
        }

        // To get the element that we want, we will use a binary search algorithm
        // instead of iterating over a for-loop. A binary search is O(std::log(n))
        // whereas searching using a loop is O(n).

        // This starts at 1 because we use the previous state later on for
        // interpolation.
        int low = 1;
        int high = controlPoints.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (controlPoints.get(mid).time < timeSeconds) {
                // This index and everything under it are less than the requested
                // timestamp. Therefore, we can discard them.
                low = mid + 1;
            } else {
                // t is at least as large as the element at this index. This means that
                // anything after it cannot be what we are looking for.
                high = mid;
            }
        }

        // High and Low should be the same.

        // The sample's timestamp is now greater than or equal to the requested
        // timestamp. If it is greater, we need to interpolate between the
        // previous state and the current state to get the exact state that we
        // want.
        final SuperstructureTrajectoryPoint sample = controlPoints.get(low);
        final SuperstructureTrajectoryPoint prevSample = controlPoints.get(low - 1);

        // If the difference in states is negligible, then we are spot on!
        if (Math.abs(sample.time - prevSample.time) < 1E-9) {
            return sample;
        }
        // Interpolate between the two states for the state that we want.
        return prevSample.interpolate(
                sample,
                (timeSeconds - prevSample.time) / (sample.time - prevSample.time));
    }

    public class SuperstructureTrajectoryPoint implements Interpolatable<SuperstructureTrajectoryPoint>{
        public SuperstructureState state;
        public double time;

        public SuperstructureTrajectoryPoint(double time, SuperstructureState state) {
            this.state = state;
            this.time = time;
        }

        @Override
        public SuperstructureTrajectoryPoint interpolate(SuperstructureTrajectoryPoint nextState, double t) {
            return new SuperstructureTrajectoryPoint(
                (this.time + nextState.time) / 2.0, 
                this.state.interpolate(nextState.state, t)
            );
        }
    }
}
