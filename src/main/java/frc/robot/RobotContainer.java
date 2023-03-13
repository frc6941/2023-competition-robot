package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoCommuteCommand;
import frc.robot.commands.AutoLoad;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveSnapRotationCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.DriveToPoseCommand;
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
                () -> mTracker.isInScore() || mTracker.isInLoad(),
                mSuperstructure::getExtensionPercentage,
                false));
        mControlBoard.getResetGyro().onTrue(new ResetGyroCommand(mDrivebase, new Rotation2d()));
        mControlBoard.getDriverController().getController().a().whileTrue(
            new DriveToPoseCommand(mDrivebase, new Pose2d(new Translation2d(5.0, 0.0), new Rotation2d()))
        );

        // mSuperstructure.setDefaultCommand(
        //     new AutoCommuteCommand(mSuperstructure, mSelector)
        //         .alongWith(new InstantCommand(mTracker::clear)).repeatedly()
        // );

        // mControlBoard.getArmIncrease().whileTrue(
        //     new InstantCommand(mControlBoard::increaseArm).repeatedly());
        // mControlBoard.getArmDecrease().whileTrue(
        //     new InstantCommand(mControlBoard::decreaseArm).repeatedly());

        // AutoLoad autoLoad = new AutoLoad(mDrivebase, mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation,
        //         mControlBoard::getArmDelta);
        // mControlBoard.getLoad().onTrue(
        //     autoLoad.getArmCommand().alongWith(new InstantCommand(mTracker::setLoad))
        //     .until(mControlBoard::getCancellation)
        //     .finallyDo((interrupted) -> mIntaker.stopIntake())
        // );

        // AutoScore autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mSelector,
        //         mControlBoard::getConfirmation, mControlBoard::getForceExtendInScore, () -> true);
        // mControlBoard.getScore().onTrue(
        //     autoScore.getArmCommand().alongWith(new InstantCommand(mTracker::setScore))
        //         .andThen(new WaitUntilNoCollision(() -> mDrivebase.getPose()))
        //         .until(mControlBoard::getCancellation)
        //         .finallyDo((interrupted) -> mIntaker.stopIntake()));

        // mControlBoard.getAutoPath().whileTrue(
        //     Commands.either(
        //         autoLoad.getDriveCommand(),
        //         autoScore.getDriveCommand(),
        //         mTracker::isInLoad));

        // mControlBoard.getSpit().whileTrue(
        //     Commands.runOnce(mIntaker::runOuttakeCone).repeatedly())
        //     .onFalse(Commands.runOnce(mIntaker::stopIntake));

        // mControlBoard.getIntake().whileTrue(
        //     Commands.runOnce(() -> mIntaker.runIntake(mSelector::getTargetGamePiece)).repeatedly())
        //     .onFalse(Commands.runOnce(mIntaker::stopIntake));

        // // Bind Operator
        // mControlBoard.getTargetMoveForward().onTrue(
        //     new InstantCommand(() -> mSelector.moveCursor(-1, 0)));
        // mControlBoard.getTargetMoveBackward().onTrue(
        //     new InstantCommand(() -> mSelector.moveCursor(1, 0)));
        // mControlBoard.getTargetMoveLeft().onTrue(
        //     new InstantCommand(() -> mSelector.moveCursor(0, 1)));
        // mControlBoard.getTargetMoveRight().onTrue(
        //     new InstantCommand(() -> mSelector.moveCursor(0, -1)));
        // mControlBoard.getApplyCursor().onTrue(
        //     new InstantCommand(() -> mSelector.applyCursorToTarget()));
        // mControlBoard.getLoadingStation().onTrue(
        //     new InstantCommand(
        //         () -> mSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION))));
        // mControlBoard.getGroundLoading().onTrue(
        //     new InstantCommand(
        //         () -> mSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND))));
        // mControlBoard.getSingleSubstation().onTrue(
        //     new InstantCommand(
        //         () -> mSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.SINGLE_SUBSTATION))));
        // mControlBoard.getGroundTipped().onTrue(
        //     new InstantCommand(
        //         () -> mSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND_TIPPED))
        //     )
        // );
        

        // mControlBoard.getArmIncrease().whileTrue(
        //     Commands.runOnce(() -> mSuperstructure.setArmPercentage(0.15), mSuperstructure).repeatedly().alongWith(Commands.runOnce(() -> mTracker.setInManual(true)))).onFalse(
        //         Commands.runOnce(() -> mSuperstructure.setAngle(mSuperstructure.getAngle()), mSuperstructure)
        //             .alongWith(new WaitUntilCommand(() -> false)).until(mControlBoard::getCancellation)
        //             .finallyDo((interrupted) -> mTracker.setInManual(false)));

        // mControlBoard.getArmDecrease().whileTrue(
        //     Commands.runOnce(() -> mSuperstructure.setArmPercentage(-0.15), mSuperstructure).repeatedly().alongWith(Commands.runOnce(() -> mTracker.setInManual(true)))).onFalse(
        //         Commands.runOnce(() -> mSuperstructure.setAngle(mSuperstructure.getAngle()), mSuperstructure)
        //             .alongWith(new WaitUntilCommand(() -> false)).until(mControlBoard::getCancellation)
        //             .finallyDo((interrupted) -> mTracker.setInManual(false)));

        // // mControlBoard.getDriverController().getController().povUp().whileTrue(
        // //     Commands.runOnce(() -> mSuperstructure.overrideProtection(true))
        // //     .andThen(
        // //         new RequestExtenderCommand(mSuperstructure, 1.07, 0.05).repeatedly()
        // //     )
        // // ).onFalse(
        // //     Commands.runOnce(() -> mSuperstructure.overrideProtection(false))
        // // );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
