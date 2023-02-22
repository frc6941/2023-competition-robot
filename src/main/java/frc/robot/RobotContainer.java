package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoCommuteCommand;
import frc.robot.commands.AutoLoadCommand;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.DriveAlongPath;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.motion.AStarPathProvider;
import frc.robot.motion.FieldObstacles;
import frc.robot.motion.PathProvider;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class RobotContainer {
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    private ArmAndExtender mSuperstructure = ArmAndExtender.getInstance();
    private Intaker mIntaker = Intaker.getInstance();
    private RobotStateEstimator mEstimator = RobotStateEstimator.getInstance();
    private TargetSelector mSelector = TargetSelector.getInstance();

    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private PathProvider mPathProvider = new AStarPathProvider();

    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(
                mDrivebase,
                mSuperstructure,
                mIntaker,
                mEstimator,
                mSelector);
        mSelector.bindButtonBoard(mControlBoard.getOperatorController());
        bindControlBoard();
    }

    private void bindControlBoard() {
        mDrivebase.setDefaultCommand(
                new DriveTeleopCommand(
                        mDrivebase,
                        mSuperstructure,
                        () -> mControlBoard.getSwerveTranslation()
                                .times(Constants.SUBSYSTEM_DRIVETRAIN.DRIVE_MAX_VELOCITY),
                        () -> mControlBoard.getSwerveRotation() * 1.5,
                        false));
        mSuperstructure.setDefaultCommand(
                new AutoCommuteCommand(mSuperstructure, mIntaker, mSelector));

        mControlBoard.getDriverController().getController().start().onTrue(
                new ResetGyroCommand(mDrivebase, new Rotation2d()));

        mControlBoard.getDriverController().getController().x().onTrue(
                new AutoLoadCommand(mSuperstructure, mIntaker, mSelector,
                        () -> mControlBoard.getConfirmation(), () -> true)
                        .until(() -> mControlBoard.getCancellation())
        );
        mControlBoard.getDriverController().getController().y().onTrue(
                new AutoScoreCommand(mDrivebase, mSuperstructure, mIntaker, mSelector,
                        () -> mControlBoard.getConfirmation(), () -> false)
                        .until(() -> mControlBoard.getCancellation()));

        mControlBoard.getDriverController().getController().leftBumper().whileTrue(
                new DriveToPoseCommand(mDrivebase, () -> mSelector.getLoadPose2d())
                .andThen(new RunCommand(() -> mDrivebase.stopMovement()))
        );
        mControlBoard.getDriverController().getController().rightBumper().whileTrue(
            new DriveAlongPath(mDrivebase, mPathProvider, () -> mSelector.getScorePose2d(), () -> FieldObstacles.getObstacles())
            .andThen(new RunCommand(() -> mDrivebase.stopMovement()))
            .unless(() -> !mIntaker.hasGamePiece())
        );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
