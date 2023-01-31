package frc.robot.states;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureStateBuilder {
    private static class Scoring {
        public static LoggedTunableNumber highRowConeAngleNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/High Row/Angle", 10.0);
        public static LoggedTunableNumber highRowConeLengthNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/High Row/Length", 1.80);
        public static LoggedTunableNumber midRowConeAngleNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Mid Row/Angle", -30.0);
        public static LoggedTunableNumber midRowConeLengthNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Mid Row/Length", 0.90);
        public static LoggedTunableNumber lowRowConeAngleNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Low Row/Angle", -70.0);
        public static LoggedTunableNumber lowRowConeLengthNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Low Row/Length", 1.00);

        public static LoggedTunableNumber highRowCubeAngleNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/High Row/Angle", 10.0);
        public static LoggedTunableNumber highRowCubeLengthNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/High Row/Length", 1.80);
        public static LoggedTunableNumber midRowCubeAngleNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Mid Row/Angle", -30.0);
        public static LoggedTunableNumber midRowCubeLengthNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Mid Row/Length", 0.90);
        public static LoggedTunableNumber lowRowCubeAngleNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Low Row/Angle", -70.0);
        public static LoggedTunableNumber lowRowCubeLengthNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Low Row/Length", 1.00);

        public static LoggedTunableNumber midRowConeAngleFar = new LoggedTunableNumber("Scoring/Cone/Far/Mid Row/Angle",
                -30.0);
        public static LoggedTunableNumber midRowConeLengthFar = new LoggedTunableNumber(
                "Scoring/Cone/Far/Mid Row/Length", 0.90);
        public static LoggedTunableNumber lowRowConeAngleFar = new LoggedTunableNumber("Scoring/Cone/Far/Low Row/Angle",
                -70.0);
        public static LoggedTunableNumber lowRowConeLengthFar = new LoggedTunableNumber(
                "Scoring/Cone/Far/Low Row/Length", 1.00);

        public static LoggedTunableNumber midRowCubeAngleFar = new LoggedTunableNumber("Scoring/Cube/Far/Mid Row/Angle",
                -30.0);
        public static LoggedTunableNumber midRowCubeLengthFar = new LoggedTunableNumber(
                "Scoring/Cube/Far/Mid Row/Length", 0.90);
        public static LoggedTunableNumber lowRowCubeAngleFar = new LoggedTunableNumber("Scoring/Cube/Far/Low Row/Angle",
                -70.0);
        public static LoggedTunableNumber lowRowCubeLengthFar = new LoggedTunableNumber(
                "Scoring/Cube/Far/Low Row/Length", 1.00);

        public static LoggedTunableNumber delta = new LoggedTunableNumber("Scoring/Delta Angle Positive", 5.0);
    }

    private static class Loading {
        public static LoggedTunableNumber shelfAngleFar = new LoggedTunableNumber("Loading/Shelf/Far/Angle", -30.0);
        public static LoggedTunableNumber shelfAngleNear = new LoggedTunableNumber("Loading/Shelf/Near/Angle", -30.0);
        public static LoggedTunableNumber shelfLengthFar = new LoggedTunableNumber("Loading/Shelf/Far/Length", -30.0);
        public static LoggedTunableNumber shelfLengthNear = new LoggedTunableNumber("Loading/Shelf/Near/Length", -30.0);

        public static LoggedTunableNumber groundAngleFar = new LoggedTunableNumber("Loading/Ground/Far/Angle", -30.0);
        public static LoggedTunableNumber groundAngleNear = new LoggedTunableNumber("Loading/Ground/Near/Angle", -30.0);
        public static LoggedTunableNumber groundLengthFar = new LoggedTunableNumber("Loading/Ground/Far/Length", -30.0);
        public static LoggedTunableNumber groundLengthNear = new LoggedTunableNumber("Loading/Ground/Near/Length",
                -30.0);
    }

    private static class Commuting {
        public static LoggedTunableNumber commuteAngleNear = new LoggedTunableNumber("Commuting/Near/Angle", -30.0);
        public static LoggedTunableNumber commuteLengthNear = new LoggedTunableNumber("Commuting/Near/Length", 0.80);
        public static LoggedTunableNumber commuteAngleFar = new LoggedTunableNumber("Commuting/Far/Angle", -30.0);
        public static LoggedTunableNumber commuteLengthFar = new LoggedTunableNumber("Commuting/Near/Length", 0.80);
    }

    public static SuperstructureState buildScoringSupertructureState(ScoringTarget target, Direction direction) {
        switch (target.getTargetGamePiece()) {
            case CUBE:
                switch (target.getScoringRow()) {
                    case HIGH:
                        return new SuperstructureState(Rotation2d.fromDegrees(Scoring.highRowCubeAngleNear.get()),
                                Scoring.highRowCubeLengthNear.get());
                    case MID:
                        if (direction == Direction.NEAR) {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.midRowCubeAngleNear.get()),
                                    Scoring.midRowCubeLengthNear.get());
                        } else {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.midRowCubeAngleFar.get()),
                                    Scoring.midRowCubeLengthFar.get());
                        }
                    case LOW:
                        if (direction == Direction.NEAR) {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.lowRowCubeAngleNear.get()),
                                    Scoring.lowRowCubeLengthNear.get());
                        } else {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.lowRowCubeAngleFar.get()),
                                    Scoring.lowRowCubeLengthFar.get());
                        }
                    default:
                        return null;
                }
            case CONE:
                switch (target.getScoringRow()) {
                    case HIGH:
                        return new SuperstructureState(Rotation2d.fromDegrees(Scoring.highRowConeAngleNear.get()),
                                Scoring.highRowConeLengthNear.get());
                    case MID:
                        if (direction == Direction.NEAR) {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.midRowConeAngleNear.get()),
                                    Scoring.midRowConeLengthNear.get());
                        } else {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.midRowConeAngleFar.get()),
                                    Scoring.midRowConeLengthFar.get());
                        }
                    case LOW:
                        if (direction == Direction.NEAR) {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.lowRowConeAngleNear.get()),
                                    Scoring.lowRowConeLengthNear.get());
                        } else {
                            return new SuperstructureState(Rotation2d.fromDegrees(Scoring.lowRowConeAngleFar.get()),
                                    Scoring.lowRowConeLengthFar.get());
                        }
                    default:
                        return null;
                }
            default:
                return null;
        }
    }

    // "Lower" the arm by an angle of delta. Near and far sides are automatically considered.
    public static SuperstructureState buildScoringSupertructureStateLowerDelta(ScoringTarget target, Direction direction) {
        SuperstructureState temp = buildScoringSupertructureState(target, direction);
        temp.armAngle = temp.armAngle.plus(Rotation2d.fromDegrees(direction == Direction.NEAR ? -Scoring.delta.get() : Scoring.delta.get()));
        return temp;
    }

    public static SuperstructureState getSuperstrucutreStateLowerDelta(SuperstructureState temp, Direction direction) {
        temp.armAngle = temp.armAngle.plus(Rotation2d.fromDegrees(direction == Direction.NEAR ? -Scoring.delta.get() : Scoring.delta.get()));
        return temp;
    }

    public static SuperstructureState buildLoadingSupertructureState(LoadingTarget target, Direction direction) {
        switch (target.getLoadingLocation()) {
            case DOUBLE_SUBSTATION_OUTER:
                if (direction == Direction.NEAR) {
                    return new SuperstructureState(Rotation2d.fromDegrees(Loading.shelfAngleNear.get()),
                            Loading.shelfLengthNear.get());
                } else {
                    return new SuperstructureState(Rotation2d.fromDegrees(Loading.shelfAngleFar.get()),
                            Loading.shelfLengthFar.get());
                }
            case DOUBLE_SUBSTATION_INNER:
                if (direction == Direction.NEAR) {
                    return new SuperstructureState(Rotation2d.fromDegrees(Loading.shelfAngleNear.get()),
                            Loading.shelfLengthNear.get());
                } else {
                    return new SuperstructureState(Rotation2d.fromDegrees(Loading.shelfAngleFar.get()),
                            Loading.shelfLengthFar.get());
                }
            case GROUND:
                if (direction == Direction.NEAR) {
                    return new SuperstructureState(Rotation2d.fromDegrees(Loading.groundAngleNear.get()),
                            Loading.groundLengthNear.get());
                } else {
                    return new SuperstructureState(Rotation2d.fromDegrees(Loading.groundAngleFar.get()),
                            Loading.groundLengthFar.get());
                }
            default:
                return null;
        }
    }

    public static SuperstructureState buildCommutingSuperstructureState(Direction direction) {
        switch (direction) {
            case NEAR:
                return new SuperstructureState(Rotation2d.fromDegrees(Commuting.commuteAngleNear.get()),
                        Commuting.commuteLengthNear.get());
            case FAR:
                return new SuperstructureState(Rotation2d.fromDegrees(Commuting.commuteAngleFar.get()),
                        Commuting.commuteLengthFar.get());
            default:
                return null;
        }
    }
}
