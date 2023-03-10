package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.TargetSelector;

public class AutoCommuteCommand extends SequentialCommandGroup {
    public AutoCommuteCommand(ArmAndExtender mSuperstructure, TargetSelector mTargetSelector) {
        addCommands(
            new RequestSuperstructureStateAutoRetract(
                mSuperstructure, () -> mTargetSelector.getCommuteSuperstructureState()
            ).unless(() -> !mSuperstructure.isHomed())
        );
    }

}
