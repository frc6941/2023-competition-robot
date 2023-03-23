package frc.robot.auto.basics;

import java.util.HashMap;
import java.util.function.Supplier;

import com.team254.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.RequestExtenderCommand;
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

public class AutoActions {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSuperstructure;
    Intaker mIntaker;
    TargetSelector mTargetSelector;

    AutoScore autoScore;
    Supplier<SuperstructureState> scoreSuperstructureStateSupplier;
    Supplier<SuperstructureState> scoreSuperstructureStateSupplierLower;
    Supplier<Pose2d> scoreDriveTargetSupplier;

    private final HashMap<String, Command> commandMapping = new HashMap<String, Command>();

    public AutoActions(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure,
            Intaker mIntaker, TargetSelector mTargetSelector) {
        this.mDrivebase = mDrivebase;
        this.mSuperstructure = mSuperstructure;
        this.mIntaker = mIntaker;
        this.mTargetSelector = mTargetSelector;

        autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mTargetSelector,
                () -> false, () -> false, () -> true);
        scoreSuperstructureStateSupplier = autoScore.getSuperstructureTargetSupplier();
        scoreSuperstructureStateSupplierLower = autoScore.getSuperstructureTargetSupplierLower();
        scoreDriveTargetSupplier = autoScore.getDrivetrainTargetSupplier();

        initMapping();
    }

    public Command groundIntake() {
        return new InstantCommand(
                mTargetSelector.getTargetGamePiece() == GamePiece.CONE ? mIntaker::runIntakeCone
                        : mIntaker::runIntakeCube)
                    .andThen(new RequestSuperstructureStateAutoRetract(mSuperstructure,
                            () -> mTargetSelector.getLoadSuperstructureState(), 60.0))
                    .andThen(new WaitUntilCommand(mIntaker::hasGamePiece))
                    .andThen(commute().alongWith(stopIntake()));
    }

    public Command prepScore() {
        return new RequestSuperstructureStateAutoRetract(mSuperstructure,
                scoreSuperstructureStateSupplier, 60.0);
    }

    public Command score() {
        return new RequestSuperstructureStateCommand(mSuperstructure,
            scoreSuperstructureStateSupplierLower)
            .unless(() -> mTargetSelector.getTargetGamePiece() == GamePiece.CUBE)
            .andThen(new WaitCommand(0.15).unless(() -> mTargetSelector.getTargetGamePiece() == GamePiece.CUBE))
            .andThen(new InstantCommand(() -> mIntaker.runOuttake(mTargetSelector::getTargetGamePiece)).alongWith(new PrintCommand("Ejecting Gamepiece!")))
            .andThen(new WaitCommand(0.2));
    }

    public Command delayZeroing(boolean value) {
        return Commands.runOnce(() -> mSuperstructure.setDelayZeroing(value));
    }

    public Command commute() {
        return new RequestSuperstructureStateAutoRetract(
            mSuperstructure,
            () -> {
                return SuperstructureStateBuilder.buildCommutingSuperstructureState(
                        mTargetSelector.getCommutingDirection());
            }
        );
    }

    public Command waitUntilRetractSafe() {
        return new WaitUntilNoCollision(() -> mDrivebase.getLocalizer().getLatestPose());
    }

    public Command waitUntilDirectionFit(Pose2d targetPose) {
        return new WaitUntilCommand(() -> Util.epsilonEquals(targetPose.getRotation().getDegrees(),
                mDrivebase.getLocalizer().getLatestPose().getRotation().getDegrees(), 30.0));
    }

    public Command waitUntilHomed() {
        return new WaitUntilCommand(mSuperstructure::isHomed);
    }

    public Command configTargetSelector(ScoringTarget scoringTarget) {
        return Commands.runOnce(() -> {
            if(scoringTarget != null) {
                mTargetSelector.setScoringTarget(scoringTarget);
            }
        });
    }

    public Command configGroundIntake() {
        return Commands.runOnce(
                () -> mTargetSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND)));
    }

    public Command stopIntake() {
        return Commands.runOnce(mIntaker::stopIntake);
    }

    public Command overrideAndGrabFarEnd() {
        return Commands.runOnce(() -> mSuperstructure.overrideProtection(true))
                .andThen(Commands.runOnce(mIntaker::runIntakeCone, mIntaker))
                .andThen(new RequestExtenderCommand(mSuperstructure, 1.03, 0.02))
                .andThen(Commands.waitSeconds(0.2)
                        .deadlineWith(Commands.waitUntil(mIntaker::hasGamePiece)))
                .andThen(Commands.runOnce(() -> mSuperstructure.overrideProtection(false)));
    }

    public Command returnToSafe() {
        return Commands.runOnce(() -> mSuperstructure.overrideProtection(false));
    }

    public Command scorePreload(ScoringTarget target) {
        if (target == null) {
            return Commands.none();
        } else {
            return Commands.sequence(
                configGroundIntake(),
                configTargetSelector(target),
                Commands.runOnce(mIntaker::runIntakeCone, mIntaker),
                waitUntilHomed(),
                overrideAndGrabFarEnd(),
                prepScore(),
                score(),
                stopIntake()
            );
        }
    }

    public Command scorePreloadFromFront(ScoringTarget target) {
        if (target == null) {
            return Commands.none();
        } else {
            return Commands.sequence(
                configGroundIntake(),
                configTargetSelector(target),
                delayZeroing(true),
                Commands.runOnce(mIntaker::runIntakeCone, mIntaker),
                delayZeroing(false),
                waitUntilHomed(),
                prepScore(),
                score(),
                stopIntake()
            );
        }
    }

    public Command balance(Pose2d startingPosition) {
        boolean enterFront = startingPosition
                .getX() < (FieldConstants.Community.chargingStationInnerX
                        + FieldConstants.Community.chargingStationOuterX) / 2.0;
        Pose2d position = new Pose2d(
            enterFront ? FieldConstants.Community.chargingStationInnerX - 0.3
                    : FieldConstants.Community.chargingStationOuterX + 0.3,
            MathUtil.clamp(startingPosition.getY(),
                    FieldConstants.Community.chargingStationRightY + 2.0,
                    FieldConstants.Community.chargingStationLeftY - 1.0),
            enterFront ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0));

        return new DriveToPoseCommand(mDrivebase, () -> position, false)
        .andThen(
            new AutoBalanceCommand(mDrivebase, enterFront)
        )
        .alongWith(commute());
    }

    public void initMapping() {
        commandMapping.put("ground intake", configGroundIntake().andThen(groundIntake()));
        commandMapping.put("prep score", prepScore());
        commandMapping.put("score", score());
        commandMapping.put("commute", commute());
    }

    public HashMap<String, Command> getCommandMapping(ScoringTarget... objectives) {
        HashMap<String, Command> baseMapping = (HashMap<String, Command>) commandMapping.clone();
        baseMapping.put("set target 1", configTargetSelector(objectives[0]));
        baseMapping.put("set target 2", configTargetSelector(objectives[1]));
        baseMapping.put("set target 3", configTargetSelector(objectives[2]));
        baseMapping.put("score preload", scorePreload(objectives[0]));

        return baseMapping;
    }
}
