package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoSortCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.FlyFly;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.BallPath;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Fly;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.StatusTracker;
import frc.robot.subsystems.TargetSelector;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    private TargetSelector mSelector = TargetSelector.getInstance();
    private StatusTracker mTracker = StatusTracker.getInstance();
    private Intaker mIntaker = Intaker.getInstance();

    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(
            // mDrivebase,
            mSelector,
            Fly.getInstance(),
            BallPath.getInstance(),
            ColorSensor.getInstance(),
            Intaker.getInstance()
            // Turret.getInstance()
            );
        bindControlBoard();
    }
    
    private void bindControlBoard() {
        mDrivebase.setDefaultCommand(
            new DriveTeleopCommand(
                mDrivebase,
                // mSuperstructure,
                mControlBoard::getSwerveTranslation,
                mControlBoard::getSwerveRotation,
                mControlBoard::getBrakeScale,
                false
            )
        );
        BallPath.getInstance().setDefaultCommand(new AutoSortCommand());
        // mControlBoard.getDriverController().getController().x().whileTrue(new DriveOnChargingStaionCommand(mDrivebase));
        mControlBoard.getDriverController().getController().a().onTrue(new ResetGyroCommand(mDrivebase, new Rotation2d()));
        // mControlBoard.getDriverController().getController().back().onTrue(new InstantCommand(()-> mDrivebase.resetPose(new Pose2d(new Translation2d(1.77, 4.72),Rotation2d.fromDegrees(180.0)))));

        mControlBoard.getDriverController().getController().x().whileTrue(
            Commands.parallel(
                Commands.runOnce(() -> {
                    BallPath.getInstance().setFeederDemand(0.50);
                    BallPath.getInstance().setTriggerDemawnd(0.50);
            }, BallPath.getInstance()).repeatedly()
            ));
        mControlBoard.getDriverController().getController().rightBumper().toggleOnTrue(
            new FlyFly(Fly.getInstance()).repeatedly()
            
        );
        mControlBoard.getDriverController().getController().y().whileTrue(
            new ParallelCommandGroup(
                Commands.runOnce(mIntaker::setForward, mIntaker),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    Commands.runOnce(() -> mIntaker.setIntakerPower(0.7))
                ),
                Commands.runOnce(() -> BallPath.getInstance().setFeederDemand(0.50), BallPath.getInstance())
            ).repeatedly()
        );
        mControlBoard.getDriverController().getController().y().whileFalse(
            Commands.parallel(
                Commands.runOnce(() -> mIntaker.setIntakerPower(0.0), mIntaker),
                Commands.runOnce(mIntaker::setReverse)
    
            )
        );

        mIntaker.setDefaultCommand(
            new ParallelCommandGroup(
                Commands.runOnce(mIntaker::setReverse),
                Commands.runOnce(() -> mIntaker.setIntakerPower(0.0), mIntaker)
            )
        );
    
        
        
        // mSuperstructure.setDefaultCommand(
        //     new AutoCommuteCommand(mSuperstructure, mIntaker, mSelector)
        //     .alongWith(new InstantCommand(mTracker::clear))
        // );
        // mControlBoard.getLoad().onTrue(
        //     new AutoLoadCommand(mSuperstructure, mIntaker, mSelector, mControlBoard::getConfirmation, () -> false)
        //     .alongWith(new InstantCommand(mTracker::setLoad))
        //     .until(mControlBoard::getCancellation)
        // );

        // AutoScore autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mSelector, () -> mControlBoard.getConfirmation(), () -> true);
        // mControlBoard.getScore().onTrue(
        //     autoScore.getArmCommand().alongWith(new InstantCommand(mTracker::setScore))
        //     .andThen(new WaitUntilNoCollision(() -> mDrivebase.getPose(), mSuperstructure, mIntaker, mSelector))
        //     .until(mControlBoard::getCancellation)
        // // );
        // mControlBoard.getAutoPath().whileTrue(
        //     Commands.either(
        //         autoScore.getDriveCommand(),
        //         Commands.either(
        //             new DriveToPoseCommand(mDrivebase, mSelector.getLoadPose2d()),
        //             new InstantCommand(),
        //             mTracker::isInLoad),
        //         mTracker::inInScore
        //     )
        // );
    }

    public UpdateManager getUpdateManager() {
        return updateManager;
    }
}
