package frc.robot.commands;

import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.TargetSelector;

public class AutoCommuteCommand extends SequentialCommandGroup {
    public AutoCommuteCommand(ArmAndExtender mSuperstructure, TargetSelector mTargetSelector) {
        addCommands(
            Commands.either(
                new RequestSuperstructureStateCommand(mSuperstructure, mTargetSelector::getCommuteSuperstructureState),
                Commands.sequence(
                    new RequestExtenderCommand(mSuperstructure, () -> mTargetSelector.getCommuteSuperstructureState().extenderLength, 0.05),
                    new RequestSuperstructureStateCommand(mSuperstructure, () -> {
                        return new SuperstructureState(mTargetSelector.getCommuteSuperstructureState().armAngle, 0.90);
                    }),
                    Commands.print("Arm Back!"),
                    new RequestSuperstructureStateCommand(mSuperstructure, mTargetSelector::getCommuteSuperstructureState).unless(() -> !Util.epsilonEquals(mTargetSelector.getCommuteSuperstructureState().armAngle.getDegrees(), mSuperstructure.getAngle(), 2.0))
                ),
                () -> {
                    return Util.epsilonEquals(mTargetSelector.getCommuteSuperstructureState().armAngle.getDegrees(), mSuperstructure.getAngle(), 2.0);
                }
            ).unless(() -> !mSuperstructure.isHomed())
        );
    }

}
