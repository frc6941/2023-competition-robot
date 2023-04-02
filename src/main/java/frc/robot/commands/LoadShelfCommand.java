package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class LoadShelfCommand extends SequentialCommandGroup {
    public LoadShelfCommand(ArmAndExtender mSuperstructure, Intaker mIntaker,
            TargetSelector mTargetSelector, BooleanSupplier confirmation) {
        addCommands(
            new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece), mIntaker),
            new RequestExtenderCommand(mSuperstructure, 1.0, 0.02),
            Commands.print("On Load"),
            Commands.parallel(
                new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece), mIntaker),
                new RequestArmCommand(mSuperstructure, () -> mTargetSelector.getLoadSuperstructureState().armAngle.getDegrees(), 2.0),
                Commands.print("Test")
            ).repeatedly().until(mIntaker::hasGamePiece),
            new RequestSuperstructureStateCommand(mSuperstructure, () -> {
                SuperstructureState temp = mTargetSelector.getLoadSuperstructureStateMinExtenderLength();
                return SuperstructureKinematics.inverseKinematics2d(
                    SuperstructureKinematics.forwardKinematics2d(
                        temp
                    ).plus(new Translation2d(0.0, 0.10))
                );
            }),
            new WaitUntilCommand(confirmation),
            new InstantCommand(mIntaker::stopIntake),
            new ManualHomeExtenderCommand(mSuperstructure),
            new RequestSuperstructureStateAutoRetract(mSuperstructure, mTargetSelector::getCommuteSuperstructureState)
        );
    }
}
