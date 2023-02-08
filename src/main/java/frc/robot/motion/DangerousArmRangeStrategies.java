package frc.robot.motion;

import org.frcteam6941.utils.Peer;
import org.frcteam6941.utils.Range;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DangerousArmRangeStrategies {
    private final List<Peer<Range, Double>> strategies = new ArrayList<>();

    public DangerousArmRangeStrategies(Range dangerousPositiveRange, Range dangerousNegativeRange) {
        strategies.add(Peer.ofRangeStrategy(dangerousPositiveRange, dangerousPositiveRange.min));
        strategies.add(Peer.ofRangeStrategy(dangerousNegativeRange, dangerousNegativeRange.max));
    }

    public Optional<Double> matchStrategy(double degree) {
        for (Peer<Range, Double> strategy : strategies) {
            if (!strategy.getKey().inRange(degree)) continue;
            return Optional.of(strategy.getValue());
        }
        return Optional.empty();
    }
}
