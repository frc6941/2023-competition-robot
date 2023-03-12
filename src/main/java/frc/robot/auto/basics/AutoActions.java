package frc.robot.auto.basics;

import java.util.List;
import java.util.function.Supplier;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.auto.AutoSelector.AUTO_START_POSITION;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.RequestSuperstructureStateAutoRetract;
import frc.robot.commands.RequestSuperstructureStateCommand;
import frc.robot.commands.WaitUntilNoCollision;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.PathPointUtil;

public class AutoActions {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSuperstructure;
    Intaker mIntaker;
    TargetSelector mTargetSelector;

    AutoScore autoScore;
    Supplier<SuperstructureState> scoreSuperstructureStateSupplier;
    Supplier<SuperstructureState> scoreSuperstructureStateSupplierLower;
    Supplier<Pose2d> scoreDriveTargetSupplier;

    private static final double xInnerMiddle = (FieldConstants.Grids.outerX + FieldConstants.Community.chargingStationInnerX) / 2.0;
    private static final double xOuter = FieldConstants.Community.chargingStationOuterX + 1.5;
    private static final double yLeftMiddle = FieldConstants.Community.leftY - 0.5;
    private static final double yRightMiddle = FieldConstants.Community.rightY + 0.5;

    private static final double xChargingStation = (FieldConstants.Community.chargingStationInnerX + FieldConstants.Community.chargingStationOuterX) / 2.0;
    private static final double yCharingStationMiddle = (FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.chargingStationRightY) / 2.0;
    private static final double yChargingStationLeft = yCharingStationMiddle + 0.1;
    private static final double yChargingStationRight = yCharingStationMiddle - 0.1;

    private static final Translation2d innerLeftTransit = new Translation2d(xInnerMiddle, yLeftMiddle);
    private static final Translation2d innerRightTransit = new Translation2d(xInnerMiddle, yRightMiddle);
    private static final Translation2d outerLeftTransit = new Translation2d(xOuter, yLeftMiddle);
    private static final Translation2d outerRightTransit = new Translation2d(xOuter, yRightMiddle);
    private static final Translation2d innerLeftChargingStationTransit = new Translation2d(xInnerMiddle, yChargingStationLeft);
    private static final Translation2d innerRightChargingStationTransit = new Translation2d(xInnerMiddle, yChargingStationRight);
    private static final Translation2d leftChargingStation = new Translation2d(xChargingStation, yChargingStationLeft);
    private static final Translation2d rightChargingStation = new Translation2d(xChargingStation, yChargingStationRight);

    private static final Pose2d innerResetPose = new Pose2d(
        new Translation2d(1.80, 5.05),
        Rotation2d.fromDegrees(180.0)
    );
    private static final Pose2d middleResetPose = new Pose2d(
        new Translation2d(1.80, 3.30),
        Rotation2d.fromDegrees(180.0)
    );
    private static final Pose2d outerResetPose = new Pose2d(
        new Translation2d(1.80, 0.50),
        Rotation2d.fromDegrees(180.0)
    );

    public AutoActions(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector) {
        this.mDrivebase = mDrivebase;
        this.mSuperstructure = mSuperstructure;
        this.mIntaker = mIntaker;
        this.mTargetSelector = mTargetSelector;

        autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mTargetSelector, () -> false, () -> true);
        scoreSuperstructureStateSupplier = autoScore.getSuperstructureTargetSupplier();
        scoreSuperstructureStateSupplierLower = autoScore.getSuperstructureTargetSupplierLower();
        scoreDriveTargetSupplier = autoScore.getDrivetrainTargetSupplier();
    }

    public Command groundIntake() {
        return new InstantCommand(mTargetSelector.getTargetGamePiece() == GamePiece.CONE ? mIntaker::runIntakeCone : mIntaker::runIntakeCube)
        .andThen(new RequestSuperstructureStateAutoRetract(
            mSuperstructure,
            () -> mTargetSelector.getLoadSuperstructureState()
        ))
        .andThen(new WaitUntilCommand(mIntaker::hasGamePiece))
        .andThen(commute());
    }

    public Command prepScore() {
        return new RequestSuperstructureStateAutoRetract(mSuperstructure, scoreSuperstructureStateSupplier);
    }

    public Command score() {
        return new RequestSuperstructureStateCommand(mSuperstructure, scoreSuperstructureStateSupplierLower).unless(() -> mTargetSelector.getTargetGamePiece() == GamePiece.CUBE)
        .andThen(new InstantCommand(() -> mIntaker.runOuttake(mTargetSelector::getTargetGamePiece)))
        .andThen(new WaitCommand(0.2));
    }

    public Command delayExtenderAction(boolean value) {
        return Commands.runOnce(() -> mSuperstructure.setDelayExtenderAction(value));
    }

    public Command commute() {
        return new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> SuperstructureStateBuilder.buildCommutingSuperstructureState(mTargetSelector.getCommutingDirection()));
    }

    private Pose2d getInnerTransitPose(boolean isLeft) {
        Rotation2d face = Rotation2d.fromDegrees(180.0);

        if(isLeft) {
            return new Pose2d(innerLeftTransit, face);
        } else {
            return new Pose2d(innerRightTransit, face);
        }
    }

    private Pose2d getOuterTransitPose(boolean isLeft) {
        Rotation2d face = Rotation2d.fromDegrees(180.0);

        if(isLeft) {
            return new Pose2d(outerLeftTransit, face);
        } else {
            return new Pose2d(outerRightTransit, face);
        }
    }

    public Command driveToInnerTransit(boolean isLeft) {
        return new DriveToPoseCommand(mDrivebase, () -> getInnerTransitPose(isLeft));
    }

    public Command pathInnerToIntake(boolean isLeft, Translation2d endIntakeTarget) {
        Pose2d innerTransit = getInnerTransitPose(isLeft);
        Pose2d outerPose = getOuterTransitPose(isLeft);
        Rotation2d intakeDirection = endIntakeTarget.minus(outerPose.getTranslation()).getAngle().plus(outerPose.getRotation());
        Pose2d outerTransformed = new Pose2d(outerPose.getTranslation(), intakeDirection);
        Pose2d intakeTransformed = new Pose2d(endIntakeTarget, intakeDirection);
        
        return new FollowTrajectory(
            mDrivebase,
            PathPointUtil.transfromPose2dToPathPoints(
                List.of(
                    innerTransit,
                    outerTransformed,
                    intakeTransformed
                )
            )
        );
    }

    public Command pathIntakeToInner(boolean isLeft, Translation2d endIntakeTarget) {
        Pose2d innerTransit = getInnerTransitPose(isLeft);
        Pose2d outerPose = getOuterTransitPose(isLeft);
        Rotation2d intakeDirection = endIntakeTarget.minus(outerPose.getTranslation()).getAngle().plus(outerPose.getRotation());
        Pose2d outerTransformed = new Pose2d(outerPose.getTranslation(), intakeDirection);
        Pose2d intakeTransformed = new Pose2d(endIntakeTarget, intakeDirection.unaryMinus());
        
        return new FollowTrajectory(
            mDrivebase,
            PathPointUtil.transfromPose2dToPathPoints(
                List.of(
                    intakeTransformed,
                    outerTransformed,
                    innerTransit
                )
            )
        );
    }

    public Command pathInnerToChargingStation(boolean isLeft) {
        Pose2d innerTransit = getInnerTransitPose(isLeft);
        return new FollowTrajectory(mDrivebase, PathPointUtil.transfromPose2dToPathPoints(
            List.of(
                innerTransit,
                new Pose2d(isLeft ? innerLeftChargingStationTransit : innerRightChargingStationTransit, innerTransit.getRotation()),
                new Pose2d(isLeft ? leftChargingStation : rightChargingStation, innerTransit.getRotation())
            )
        ));
    }

    public Command driveToScoreTarget() {
        return new DriveToPoseCommand(mDrivebase, scoreDriveTargetSupplier);
    }

    public Command waitUntilRetractSafe() {
        return new WaitUntilNoCollision(mDrivebase.getLocalizer()::getLatestPose, mSuperstructure, mIntaker, mTargetSelector);
    }

    public Command waitUntilDirectionFit(Pose2d targetPose) {
        return new WaitUntilCommand(() -> Util.epsilonEquals(targetPose.getRotation().getDegrees(), mDrivebase.getLocalizer().getLatestPose().getRotation().getDegrees(), 30.0));
    }

    public Command waitUntilHomed() {
        return new WaitUntilCommand(mSuperstructure::isHomed);
    }

    public Command configTargetSelector(ScoringTarget scoringTarget) {
        return Commands.runOnce(() -> mTargetSelector.setScoringTarget(scoringTarget));
    }

    public Command configGroundIntake() {
        return Commands.runOnce(() -> mTargetSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND)));
    }

    public Command resetPose(AUTO_START_POSITION startPosition) {
        switch(startPosition) {
            case INNER:
                return Commands.runOnce(() -> mDrivebase.resetPose(AllianceFlipUtil.apply(innerResetPose)));
            case OUTER:
                return Commands.runOnce(() -> mDrivebase.resetPose(AllianceFlipUtil.apply(outerResetPose)));
            case CENTER:
                return Commands.runOnce(() -> mDrivebase.resetPose(AllianceFlipUtil.apply(middleResetPose)));
            default:
                return Commands.none();
        }
    }


    public Command scorePreload(boolean isLeft, ScoringTarget target) {
        if(target == null) {
            return Commands.none();
        } else {
            return Commands.sequence(
                delayExtenderAction(true),
                configGroundIntake(),
                configTargetSelector(target),
                Commands.runOnce(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece)),
                Commands.waitSeconds(0.5),
                delayExtenderAction(false),
                waitUntilHomed(),
                prepScore(),
                score(),
                driveToInnerTransit(isLeft)
            );
        }
    }

    public Command intakeAndScore(boolean isLeft, ScoringTarget target, Translation2d intakePosition) {
        if(target == null) {
            return Commands.none();
        } else {
            return Commands.sequence(
                configGroundIntake(),
                driveToInnerTransit(isLeft),
                configTargetSelector(target),
                pathInnerToIntake(isLeft, intakePosition)
                .deadlineWith(
                    groundIntake()
                ),
                pathIntakeToInner(isLeft, intakePosition)
                .alongWith(
                    prepScore()
                ),
                driveToScoreTarget(),
                score(),
                driveToInnerTransit(isLeft)
            );
        }
    }

    public Command balance(boolean isLeft) {
        return Commands.sequence(
            driveToInnerTransit(isLeft).alongWith(commute()),
            pathInnerToChargingStation(isLeft),
            new AutoBalanceCommand(mDrivebase)
        );
    }
}
