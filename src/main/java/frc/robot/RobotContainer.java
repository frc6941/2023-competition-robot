package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoCommuteCommand;
import frc.robot.commands.AutoLoad;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.LoadGroundCommand;
import frc.robot.commands.LoadShelfCommand;
import frc.robot.commands.ManualHomeExtenderCommand;
import frc.robot.commands.RequestSuperstructureStateAutoRetract;
import frc.robot.commands.RequestSuperstructureStateCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.WaitUntilNoCollision;
import frc.robot.controlboard.ControlBoard;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.StatusTracker;
import frc.robot.subsystems.TargetSelector;

public class RobotContainer {
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    private ArmAndExtender mSuperstructure = ArmAndExtender.getInstance();
    private Intaker mIntaker = Intaker.getInstance();
    private RobotStateEstimator mEstimator = RobotStateEstimator.getInstance();
    private TargetSelector mSelector = TargetSelector.getInstance();
    private StatusTracker mTracker = StatusTracker.getInstance();

    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(
                mDrivebase,
                mSuperstructure,
                mIntaker,
                mEstimator,
                mSelector,
                mTracker);
        bindControlBoard();
    }

    private void bindControlBoard() {
        // Bind Driver
        mDrivebase.setDefaultCommand(
                new DriveTeleopCommand(
                        mDrivebase,
                        mSuperstructure,
                        mControlBoard::getSwerveTranslation,
                        mControlBoard::getSwerveRotation,
                        () -> mTracker.isInLoad(),
                        () -> mTracker.isInScore(),
                        () -> mTracker.isSpeedRedctionActivate(),
                        () -> mTracker.isYCancelActivate(),
                        mSuperstructure::getExtensionPercentage,
                        false));
        mControlBoard.getResetGyro().onTrue(new ResetGyroCommand(mDrivebase, new Rotation2d()));

        mSuperstructure.setDefaultCommand(
                new AutoCommuteCommand(mSuperstructure, mSelector)
                        .alongWith(new InstantCommand(mTracker::clear))
                        .repeatedly());

        AutoLoad autoLoad = new AutoLoad(mDrivebase, mSuperstructure, mIntaker, mSelector,
            mControlBoard::getConfirmation);
        mControlBoard.getLoad()
            .and(() -> mSelector.getLoadingTarget().getLoadingLocation() == LOADING_LOCATION.DOUBLE_SUBSTATION)
            .onTrue(
                new LoadShelfCommand(mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation)
                    .alongWith(new InstantCommand(mTracker::setLoad))
                    .until(mControlBoard::getCancellation)
                    .finallyDo((interrupted) -> {
                        mIntaker.stopIntake();
                    }));

        mControlBoard.getLoad()
            .and(() -> mSelector.getLoadingTarget().getLoadingLocation() == LOADING_LOCATION.GROUND
                    || mSelector.getLoadingTarget().getLoadingLocation() == LOADING_LOCATION.GROUND_TIPPED)
            .whileTrue(
                new LoadGroundCommand(mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation)
                    .alongWith(new InstantCommand(mTracker::setLoad))
                    .finallyDo((interrupted) -> {
                        Commands.sequence(
                            new InstantCommand(mIntaker::stopIntake),
                            new RequestSuperstructureStateCommand(mSuperstructure, () -> {
                                SuperstructureState intakeState = mSelector
                                        .getLoadSuperstructureState();
                                Translation2d tempState = SuperstructureKinematics
                                        .forwardKinematics2d(intakeState);
                                Translation2d raiseUp = new Translation2d(
                                        tempState.getX() < 0.0 ? -0.10 : 0.10, 0.20);
                                return SuperstructureKinematics
                                        .inverseKinematics2d(tempState.plus(raiseUp));
                            }),
                            new WaitCommand(0.3),
                            new RequestSuperstructureStateAutoRetract(mSuperstructure,
                                    () -> mSelector.getCommuteSuperstructureState()))
                            .unless(() -> !mIntaker.hasGamePiece())
                            .schedule();
            }));

        AutoScore autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mSelector,
            mControlBoard::getConfirmation, mTracker::getForceExtend, () -> true);
        mControlBoard.getScore().onTrue(
            autoScore.getArmCommand().alongWith(new InstantCommand(mTracker::setScore))
                .andThen(new WaitUntilNoCollision(() -> mDrivebase.getPose()))
                .until(mControlBoard::getCancellation)
                .finallyDo((interrupted) -> mIntaker.stopIntake()));

        mControlBoard.getAutoPath().whileTrue(
            new InstantCommand(() -> mDrivebase.setLockHeading(true)).andThen(
                Commands.either(
                    autoLoad.getDriveCommand().alongWith(Commands.runOnce(() -> mTracker.setYCancel(true))),
                    autoScore.getDriveCommand().alongWith(Commands.runOnce(mTracker::enableSpeedLimit)),
                    mTracker::isInLoad)))
            .onFalse(new InstantCommand(() -> {
                mDrivebase.setLockHeading(false);
                mTracker.setYCancel(false);
            }));

        mControlBoard.getSpit().whileTrue(
            Commands.run(() -> mIntaker.runOuttake(mSelector::getTargetGamePiece)))
            .onFalse(Commands.runOnce(mIntaker::stopIntake));

        mControlBoard.getIntake().whileTrue(
            Commands.run(() -> mIntaker.runIntake(mSelector::getTargetGamePiece)))
            .onFalse(Commands.runOnce(mIntaker::stopIntake));
        mControlBoard.getDriverController().getController().povUp().whileTrue(
            new AutoBalanceCommand(mDrivebase, true));

        mControlBoard.getDriverController().getController().povDown().whileTrue(
            new AutoBalanceCommand(mDrivebase, false));
        mControlBoard.getForceExtendInScore().onTrue(
            Commands.runOnce(mTracker::toggleForceExtend));
        mControlBoard.getDriverController().getController().back().onTrue(
            new ManualHomeExtenderCommand(mSuperstructure));

        mControlBoard.getArmIncrease().whileTrue(
            Commands.runOnce(() -> mSuperstructure.setArmPercentage(0.10),
                mSuperstructure).repeatedly().alongWith(Commands.runOnce(() -> mTracker.setInManual(true))))
            .onFalse(
                Commands.runOnce(() -> mSuperstructure.setAngle(mSuperstructure.getAngle()), mSuperstructure)
                    .alongWith(new WaitUntilCommand(() -> false)).until(mControlBoard::getCancellation)
                        .finallyDo((interrupted) -> mTracker.setInManual(false)));

        mControlBoard.getArmDecrease().whileTrue(
            Commands.runOnce(() -> mSuperstructure.setArmPercentage(-0.10),
                mSuperstructure).repeatedly().alongWith(Commands.runOnce(() -> mTracker.setInManual(true))))
            .onFalse(
                Commands.runOnce(() -> mSuperstructure.setAngle(mSuperstructure.getAngle()), mSuperstructure)
                    .alongWith(new WaitUntilCommand(() -> false)).until(mControlBoard::getCancellation)
                    .finallyDo((interrupted) -> mTracker.setInManual(false)));

        // Bind Operator
        mControlBoard.getSetHighTarget().onTrue(
            Commands.runOnce(() -> mSelector.setTargetRow(SCORING_ROW.HIGH)));

        mControlBoard.getSetMidTarget().onTrue(
            Commands.runOnce(() -> mSelector.setTargetRow(SCORING_ROW.MID)));

        mControlBoard.getSetLowTarget().onTrue(
            Commands.runOnce(() -> mSelector.setTargetRow(SCORING_ROW.LOW)));

        mControlBoard.getSetDoubleSubstation().onTrue(
            Commands.runOnce(
                () -> {
                    mSelector.setTargetGamePiece(GamePiece.CONE);
                    mSelector.setLoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION);
                }));

        mControlBoard.getGroundAlongWithSetCone().onTrue(
            Commands.runOnce(
                () -> {
                    mSelector.setTargetGamePiece(GamePiece.CONE);
                    mSelector.setLoadingTarget(LOADING_LOCATION.GROUND);
                }));

        mControlBoard.getGroundAlongWithSetCube().onTrue(
                Commands.runOnce(
                        () -> {
                            mSelector.setTargetGamePiece(GamePiece.CUBE);
                            mSelector.setLoadingTarget(LOADING_LOCATION.GROUND);
                        }));

        mControlBoard.getGroundTipped().onTrue(
                Commands.runOnce(
                        () -> {
                            mSelector.setTargetGamePiece(GamePiece.CONE);
                            mSelector.setLoadingTarget(LOADING_LOCATION.GROUND_TIPPED);
                        }));

        mControlBoard.togggleCommuteNear().onTrue(
                Commands.runOnce(
                        mSelector::toggleCanCommuteNear));

        mControlBoard.getOperatorController().getController().back().onTrue(
                new ManualHomeExtenderCommand(mSuperstructure));
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
