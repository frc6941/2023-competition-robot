package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class LoadShelfCommand extends SequentialCommandGroup {
    public LoadShelfCommand(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker,
            TargetSelector mTargetSelector, BooleanSupplier confirmation) {
        addCommands(
            Commands.sequence(
                new RequestSuperstructureStateAutoRetract(mSuperstructure, mTargetSelector::getLoadSuperstructureStateMinExtenderLength),
                new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece), mIntaker),
                new RequestExtenderCommand(mSuperstructure, 1.2, 0.02).unless(mIntaker::hasGamePiece),
                new WaitUntilCommand(confirmation).unless(mIntaker::hasGamePiece),
                new InstantCommand(mIntaker::stopIntake),
                new RequestExtenderCommand(mSuperstructure, 0.885, 0.02),
                new WaitUntilNoCollision(() -> mDrivebase.getLocalizer().getLatestPose()),
                new RequestSuperstructureStateAutoRetract(mSuperstructure, mTargetSelector::getCommuteSuperstructureState)
            ).unless(mIntaker::hasGamePiece)
        );
    }
}
