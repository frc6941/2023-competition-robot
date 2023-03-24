package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoCommuteCommand;
import frc.robot.commands.AutoLoad;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.WaitUntilNoCollision;
import frc.robot.controlboard.ControlBoard;
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
                mTracker
        );
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
                false
            )
        );
        mControlBoard.getResetGyro().onTrue(new ResetGyroCommand(mDrivebase, new Rotation2d()));

        mSuperstructure.setDefaultCommand(
            new AutoCommuteCommand(mSuperstructure, mSelector)
                .alongWith(new InstantCommand(mTracker::clear))
                .repeatedly()
        );

        AutoLoad autoLoad = new AutoLoad(mDrivebase, mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation);
        mControlBoard.getLoad().onTrue(
            autoLoad.getArmCommand().alongWith(new InstantCommand(mTracker::setLoad))
            .until(mControlBoard::getCancellation)
            .finallyDo((interrupted) -> mIntaker.stopIntake())
        );

        AutoScore autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mSelector,
                mControlBoard::getConfirmation, mControlBoard::getForceExtendInScore, () -> true);
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
                    mTracker::isInLoad
                )
            )
        ).onFalse(new InstantCommand(() -> {
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
            new AutoBalanceCommand(mDrivebase, true)
        );

        mControlBoard.getDriverController().getController().povDown().whileTrue(
            new AutoBalanceCommand(mDrivebase, false)
        );
        

        // Bind Operator
        mControlBoard.getTargetMoveForward().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(-1, 0)).ignoringDisable(true)
        );
        mControlBoard.getTargetMoveBackward().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(1, 0)).ignoringDisable(true)
        );
        mControlBoard.getTargetMoveLeft().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(0, 1)).ignoringDisable(true)
        );
        mControlBoard.getTargetMoveRight().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(0, -1)).ignoringDisable(true)
        );
        mControlBoard.getApplyCursor().onTrue(
            new InstantCommand(() -> mSelector.applyCursorToTarget()).ignoringDisable(true)
        );
        mControlBoard.getLoadingTargetIncrease().onTrue(
            new InstantCommand(() -> mSelector.moveLoadingTarget(2)).ignoringDisable(true)
        );
        mControlBoard.getLoadingTargetDecrease().onTrue(
            new InstantCommand(() -> mSelector.moveLoadingTarget(-2)).ignoringDisable(true)
        );
        mControlBoard.getCanCommuteNear().onTrue(
            new InstantCommand(mSelector::toggleCanCommuteNear)
        ).onFalse(
            Commands.none()
        );
        mControlBoard.getSetCone().onTrue(
            Commands.runOnce(mSelector::setCone)
        );
        mControlBoard.getSetCube().onTrue(
            Commands.runOnce(mSelector::setCube)
        );
        

        mControlBoard.getArmIncrease().whileTrue(
            Commands.runOnce(() -> mSuperstructure.setArmPercentage(0.10), mSuperstructure).repeatedly().alongWith(Commands.runOnce(() -> mTracker.setInManual(true)))).onFalse(
                Commands.runOnce(() -> mSuperstructure.setAngle(mSuperstructure.getAngle()), mSuperstructure)
                    .alongWith(new WaitUntilCommand(() -> false)).until(mControlBoard::getCancellation)
                    .finallyDo((interrupted) -> mTracker.setInManual(false)));

        mControlBoard.getArmDecrease().whileTrue(
            Commands.runOnce(() -> mSuperstructure.setArmPercentage(-0.10), mSuperstructure).repeatedly().alongWith(Commands.runOnce(() -> mTracker.setInManual(true)))).onFalse(
                Commands.runOnce(() -> mSuperstructure.setAngle(mSuperstructure.getAngle()), mSuperstructure)
                    .alongWith(new WaitUntilCommand(() -> false)).until(mControlBoard::getCancellation)
                    .finallyDo((interrupted) -> mTracker.setInManual(false)));
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
