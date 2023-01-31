package frc.robot.states;

import org.frcteam6328.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureStateBuilder {
    private static class Scoring {
        public static LoggedTunableNumber highRowConeAngleNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/High Row/Angle");
        public static LoggedTunableNumber highRowConeLengthNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/High Row/Length");
        public static LoggedTunableNumber midRowConeAngleNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Mid Row/Angle");
        public static LoggedTunableNumber midRowConeLengthNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Mid Row/Length");
        public static LoggedTunableNumber lowRowConeAngleNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Low Row/Angle");
        public static LoggedTunableNumber lowRowConeLengthNear = new LoggedTunableNumber(
                "Scoring/Cone/Near/Low Row/Length");

        public static LoggedTunableNumber highRowCubeAngleNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/High Row/Angle");
        public static LoggedTunableNumber highRowCubeLengthNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/High Row/Length");
        public static LoggedTunableNumber midRowCubeAngleNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Mid Row/Angle");
        public static LoggedTunableNumber midRowCubeLengthNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Mid Row/Length");
        public static LoggedTunableNumber lowRowCubeAngleNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Low Row/Angle");
        public static LoggedTunableNumber lowRowCubeLengthNear = new LoggedTunableNumber(
                "Scoring/Cube/Near/Low Row/Length");

        public static LoggedTunableNumber midRowConeAngleFar = new LoggedTunableNumber("Scoring/Cone/Far/Mid Row/Angle",
                -30.0);
        public static LoggedTunableNumber midRowConeLengthFar = new LoggedTunableNumber(
                "Scoring/Cone/Far/Mid Row/Length");
        public static LoggedTunableNumber lowRowConeAngleFar = new LoggedTunableNumber("Scoring/Cone/Far/Low Row/Angle");
        public static LoggedTunableNumber lowRowConeLengthFar = new LoggedTunableNumber(
                "Scoring/Cone/Far/Low Row/Length");

        public static LoggedTunableNumber midRowCubeAngleFar = new LoggedTunableNumber("Scoring/Cube/Far/Mid Row/Angle");
        public static LoggedTunableNumber midRowCubeLengthFar = new LoggedTunableNumber(
                "Scoring/Cube/Far/Mid Row/Length");
        public static LoggedTunableNumber lowRowCubeAngleFar = new LoggedTunableNumber("Scoring/Cube/Far/Low Row/Angle");
        public static LoggedTunableNumber lowRowCubeLengthFar = new LoggedTunableNumber(
                "Scoring/Cube/Far/Low Row/Length");

        public static LoggedTunableNumber delta = new LoggedTunableNumber("Scoring/Delta Angle Positive");
    }

    private static class Loading {
        public static LoggedTunableNumber shelfAngleFar = new LoggedTunableNumber("Loading/Shelf/Far/Angle");
        public static LoggedTunableNumber shelfAngleNear = new LoggedTunableNumber("Loading/Shelf/Near/Angle");
        public static LoggedTunableNumber shelfLengthFar = new LoggedTunableNumber("Loading/Shelf/Far/Length");
        public static LoggedTunableNumber shelfLengthNear = new LoggedTunableNumber("Loading/Shelf/Near/Length");

        public static LoggedTunableNumber groundAngleFar = new LoggedTunableNumber("Loading/Ground/Far/Angle");
        public static LoggedTunableNumber groundAngleNear = new LoggedTunableNumber("Loading/Ground/Near/Angle");
        public static LoggedTunableNumber groundLengthFar = new LoggedTunableNumber("Loading/Ground/Far/Length");
        public static LoggedTunableNumber groundLengthNear = new LoggedTunableNumber("Loading/Ground/Near/Length");
    }

    private static class Commuting {
        public static LoggedTunableNumber commuteAngleNear = new LoggedTunableNumber("Commuting/Near/Angle");
        public static LoggedTunableNumber commuteLengthNear = new LoggedTunableNumber("Commuting/Near/Length");
        public static LoggedTunableNumber commuteAngleFar = new LoggedTunableNumber("Commuting/Far/Angle");
        public static LoggedTunableNumber commuteLengthFar = new LoggedTunableNumber("Commuting/Far/Length");
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
        double armAngleDegree = temp.armAngle.getDegrees() + (direction == Direction.NEAR ? -Scoring.delta.get() : Scoring.delta.get());
        
        return new SuperstructureState(Rotation2d.fromDegrees(armAngleDegree), temp.extenderLength);
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

    static {
        /* Scoring */
        // High Row - Near Side
        Scoring.highRowConeAngleNear.initDefault(10);
        Scoring.highRowConeLengthNear.initDefault(1.40);
        Scoring.highRowCubeAngleNear.initDefault(0);
        Scoring.highRowCubeLengthNear.initDefault(1.40);

        // Mid Row - Near Side
        Scoring.midRowConeAngleNear.initDefault(-20.0);
        Scoring.midRowConeLengthNear.initDefault(0.90);
        Scoring.midRowCubeAngleNear.initDefault(-25.0);
        Scoring.midRowConeLengthNear.initDefault(0.80);

        // Low Row - Near Side
        Scoring.lowRowConeAngleNear.initDefault(-65.0);
        Scoring.lowRowConeLengthNear.initDefault(1.20);
        Scoring.lowRowCubeAngleNear.initDefault(-65.0);
        Scoring.lowRowConeLengthNear.initDefault(1.20);

        // Mid Row - Far Side
        Scoring.midRowConeAngleFar.initDefault(200);
        Scoring.midRowConeLengthFar.initDefault(1.40);
        Scoring.midRowCubeAngleFar.initDefault(205);
        Scoring.midRowConeLengthFar.initDefault(1.30);

        // Low Row - Far Side
        Scoring.lowRowConeAngleFar.initDefault(245);
        Scoring.lowRowConeLengthFar.initDefault(1.40);
        Scoring.lowRowCubeAngleFar.initDefault(245);
        Scoring.lowRowConeLengthFar.initDefault(1.40);

        // Delta
        Scoring.delta.initDefault(5);



        /* Loading */
        // Loading - Double Substation - Near Side
        Loading.shelfAngleNear.initDefault(0);
        Loading.shelfLengthNear.initDefault(1.40);

        // Loading - Double Substation - Far Side
        Loading.shelfAngleFar.initDefault(180.0);
        Loading.shelfLengthFar.initDefault(1.40);

        // Loading - Ground - Near Side
        Loading.groundAngleNear.initDefault(-75.0);
        Loading.groundLengthNear.initDefault(1.00);

        // Loading - Ground - Far Side
        Loading.groundAngleFar.initDefault(220.0);
        Loading.groundLengthFar.initDefault(1.40);



        /* Commuting */
        Commuting.commuteAngleNear.initDefault(-90.0);
        Commuting.commuteLengthNear.initDefault(0.80);
        Commuting.commuteAngleFar.initDefault(255.0);
        Commuting.commuteLengthFar.initDefault(0.80);
    }
}
