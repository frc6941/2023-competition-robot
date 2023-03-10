package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoCommuteCommand;
import frc.robot.commands.AutoLoad;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.WaitUntilNoCollision;
import frc.robot.controlboard.ControlBoard;
import frc.robot.states.LoadingTarget;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
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
                mControlBoard::getBrakeScale,
                false
            )
        );
        mControlBoard.getResetGyro().onTrue(new ResetGyroCommand(mDrivebase, new Rotation2d()));

        mSuperstructure.setDefaultCommand(
            new AutoCommuteCommand(mSuperstructure, mIntaker, mSelector)
            .alongWith(new InstantCommand(mTracker::clear))
        );

        mControlBoard.getArmIncrease().whileTrue(
            new InstantCommand(mControlBoard::increaseArm).repeatedly()
        );
        mControlBoard.getArmDecrease().whileTrue(
            new InstantCommand(mControlBoard::decreaseArm).repeatedly()
        );

        AutoLoad autoLoad = new AutoLoad(mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation, mControlBoard::getArmDelta);
        mControlBoard.getLoad().onTrue(
            Commands.sequence(
                Commands.runOnce(mControlBoard::clearArmDelta),
                autoLoad.getArmCommand().alongWith(new InstantCommand(mTracker::setLoad))
            ).until(mControlBoard::getCancellation)
        );

        AutoScore autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation, () -> true);
        mControlBoard.getScore().onTrue(
            autoScore.getArmCommand().alongWith(new InstantCommand(mTracker::setScore))
            .andThen(new WaitUntilNoCollision(() -> mDrivebase.getPose(), mSuperstructure, mIntaker, mSelector))
            .until(mControlBoard::getCancellation)
        );
        
        mControlBoard.getAutoPath().whileTrue(
            Commands.either(
                autoScore.getDriveCommand(),
                Commands.either(
                    autoLoad.getDriveCommand(),
                    Commands.none(),
                    mTracker::isInLoad),
                mTracker::inInScore
            )
        );

        mIntaker.setDefaultCommand(
            Commands.runOnce(mIntaker::stopIntake, mIntaker)
        );
        
        mControlBoard.getSpit().whileTrue(
            new InstantCommand(mIntaker::runOuttake, mIntaker).repeatedly()
        )
        .onFalse(
            new InstantCommand(mIntaker::stopIntake)
        );

        mControlBoard.getIntake().whileTrue(
            new InstantCommand(() -> mIntaker.runIntake(mSelector::getTargetGamePiece), mIntaker).repeatedly()
        )
        .onFalse(
            new InstantCommand(mIntaker::stopIntake)
        );
        

        // Bind Operator
        mControlBoard.getTargetMoveForward().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(-1, 0))
        );
        mControlBoard.getTargetMoveBackward().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(1, 0))
        );
        mControlBoard.getTargetMoveLeft().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(0, 1))
        );
        mControlBoard.getTargetMoveRight().onTrue(
            new InstantCommand(() -> mSelector.moveCursor(0, -1))
        );
        mControlBoard.getApplyCursor().onTrue(
            new InstantCommand(() -> mSelector.applyCursorToTarget())
        );
        mControlBoard.getLoadingStation().onTrue(
            new InstantCommand(() -> mSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION)))
        );
        mControlBoard.getGroundLoading().onTrue(
            new InstantCommand(() -> mSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND)))
        );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
