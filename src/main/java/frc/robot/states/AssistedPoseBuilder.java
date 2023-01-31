package frc.robot.states;

import org.frcteam6328.utils.LoggedTunableNumber;
import org.frcteam6941.control.DirectionalPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;

public class AssistedPoseBuilder {
    private static LoggedTunableNumber loadingLeftQuaterY = new LoggedTunableNumber("Loading/Left Y",
            FieldConstants.LoadingZone.leftQuaterY);
    private static LoggedTunableNumber loadingRightQuaterY = new LoggedTunableNumber("Loading/Right Y",
            FieldConstants.LoadingZone.rightQuaterY);
    private static LoggedTunableNumber scoringLineX = new LoggedTunableNumber("Scoring/Line X",
            FieldConstants.Grids.outerX + 0.50);

    public static DirectionalPose2d buildLoadingDirectionalPose2d(LoadingTarget loadingTarget, Direction direction) {
        double directionDegrees;
        if (direction == Direction.NEAR) {
            directionDegrees = 0.0;
        } else {
            directionDegrees = 180.0;
        }

        switch (loadingTarget.getLoadingLocation()) {
            case DOUBLE_SUBSTATION_OUTER:
                return new DirectionalPose2d(
                        FieldConstants.allianceFlip(
                                new Pose2d(
                                        new Translation2d(0.0, loadingLeftQuaterY.get()),
                                        Rotation2d.fromDegrees(directionDegrees))),
                        false, true, true);
            case DOUBLE_SUBSTATION_INNER:
                return new DirectionalPose2d(
                        FieldConstants.allianceFlip(
                                new Pose2d(
                                        new Translation2d(0.0, loadingRightQuaterY.get()),
                                        Rotation2d.fromDegrees(directionDegrees))),
                        false, true, true);
            case GROUND:
                return null;
            default:
                return null;
        }
    }

    public static DirectionalPose2d buildScoringDirectionalPose2d(ScoringTarget scoringTarget, Direction direction) {
        double directionDegrees;
        if (direction == Direction.NEAR) {
            directionDegrees = 180.0;
        } else {
            directionDegrees = 0.0;
        }

        double targetY = FieldConstants.Grids.complexLowTranslations[scoringTarget.getPosition()].getY();
        return new DirectionalPose2d(
                FieldConstants.allianceFlip(
                        new Pose2d(
                                new Translation2d(scoringLineX.get(), targetY),
                                Rotation2d.fromDegrees(directionDegrees))),
                true, true, true);
    }
}
