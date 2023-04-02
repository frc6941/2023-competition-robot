package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;

public class RequestSuperstructureStateAutoRetract extends SequentialCommandGroup{
    public RequestSuperstructureStateAutoRetract(ArmAndExtender armAndExtender, Supplier<SuperstructureState> targetState, double epsilon) {
        addCommands(
            new RequestExtenderCommand(armAndExtender, 0.90, 0.5),
            Commands.either(
                new RequestArmCommand(armAndExtender, () -> targetState.get().armAngle.getDegrees(), 2.0),
                new RequestArmCommand(armAndExtender, () -> targetState.get().armAngle.getDegrees(), epsilon),
                () -> Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_NEGATIVE.inRange(targetState.get().armAngle.getDegrees())
                || Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_POSITIVE.inRange(targetState.get().armAngle.getDegrees())
            ),
            new RequestSuperstructureStateCommand(armAndExtender, targetState)
        );
    }

    public RequestSuperstructureStateAutoRetract(ArmAndExtender armAndExtender, Supplier<SuperstructureState> targetState) {
        addCommands(
            new RequestExtenderCommand(armAndExtender, 0.90, 0.05),
            Commands.either(
                new RequestArmCommand(armAndExtender, () -> targetState.get().armAngle.getDegrees(), 2.0),
                new RequestArmCommand(armAndExtender, () -> targetState.get().armAngle.getDegrees(), 2.0),
                () -> Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_NEGATIVE.inRange(targetState.get().armAngle.getDegrees())
                || Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_POSITIVE.inRange(targetState.get().armAngle.getDegrees())
            ),
            new RequestSuperstructureStateCommand(armAndExtender, targetState)
        );
    }
}
