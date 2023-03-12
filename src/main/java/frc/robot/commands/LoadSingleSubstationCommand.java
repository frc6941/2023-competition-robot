package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class LoadSingleSubstationCommand extends SequentialCommandGroup {
    public LoadSingleSubstationCommand(ArmAndExtender mSuperstructure, Intaker mIntaker,
            TargetSelector mTargetSelector) {
        addCommands(
            Commands.sequence(
                new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> mTargetSelector.getLoadSuperstructureState()),
                new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece), mIntaker),
                new WaitUntilCommand(mIntaker::hasGamePiece),
                new InstantCommand(mIntaker::stopIntake),
                new RequestExtenderCommand(mSuperstructure, 0.885, 0.05),
                new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> mTargetSelector.getCommuteSuperstructureState())
            ).unless(mIntaker::hasGamePiece)
        );
    }
}
