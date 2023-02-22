package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.states.GamePiece;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker,
            TargetSelector mTargetSelector, BooleanSupplier confirmation, BooleanSupplier shouldAutoScore) {
        addCommands(
            new RequestSuperstructureStateAutoRetract(mSuperstructure,
                    () -> mTargetSelector.getHairTriggerSuperstructureState()).unless(() -> mDrivebase.getLocalizer().getLatestPose().getTranslation().minus(mTargetSelector.getScorePose2d().getTranslation()).getNorm() < 1.0)
            .andThen(new WaitUntilCommand(() -> mDrivebase.getLocalizer().getLatestPose().getTranslation().minus(mTargetSelector.getScorePose2d().getTranslation()).getNorm() < 1.0))
            .andThen(new RequestSuperstructureStateAutoRetract(mSuperstructure,
            () -> mTargetSelector.getScoreSuperstructureState()))
            .andThen(new WaitUntilCommand(confirmation))
            .andThen(
                new ConditionalCommand(
                    new RequestSuperstructureStateCommand(mSuperstructure, () -> mTargetSelector.getScoreLowerDelSuperstructureState())
                    .andThen(new WaitCommand(0.2))
                    .andThen(new InstantCommand(() -> mIntaker.setIntakerPower(Constants.SUBSYSTEM_INTAKE.OUTTAKING_SLOW_PERCENTAGE)))
                    .andThen(new WaitCommand(0.5))
                    .andThen(new RequestSuperstructureStateCommand(mSuperstructure, () -> mTargetSelector.getScoreSuperstructureState())),
                    
                    new InstantCommand(() -> mIntaker.setIntakerPower(Constants.SUBSYSTEM_INTAKE.OUTTAKING_FAST_PERCENTAGE)),
                    () -> mTargetSelector.getTargetGamePiece() == GamePiece.CONE))
            .andThen(new WaitUntilCommand(confirmation))
            .andThen(new InstantCommand(() -> mIntaker.setIntakerPower(0.0)))
            .andThen(new WaitUntilCommand(() -> mDrivebase.getLocalizer().getLatestPose().getX() > 2.0))
            .andThen(new RequestSuperstructureStateAutoRetract(mSuperstructure,
                    () -> mTargetSelector.getCommuteSuperstructureState()))
        );
    }
}
