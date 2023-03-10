package frc.robot.commands;

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

public class LoadGroundCommand extends SequentialCommandGroup{
    public LoadGroundCommand(ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector) {
        addCommands(
            Commands.sequence(
                new RequestSuperstructureStateAutoRetract(mSuperstructure, mTargetSelector::getLoadSuperstructureState),
                new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece)),
                new WaitUntilCommand(() -> mIntaker.hasGamePiece()),
                new InstantCommand(mIntaker::stopIntake),
                new RequestSuperstructureStateCommand(mSuperstructure, () -> {
                    SuperstructureState intakeState = mTargetSelector.getLoadSuperstructureState();
                    Translation2d tempState = SuperstructureKinematics.forwardKinematics2d(intakeState);
                    Translation2d raiseUp = new Translation2d(tempState.getX() < 0.0 ? -0.20 : 0.20, 0.40);
                    return SuperstructureKinematics.inverseKinematics2d(tempState.plus(raiseUp));
                }),
                new RequestSuperstructureStateAutoRetract(mSuperstructure,
                        () -> mTargetSelector.getCommuteSuperstructureState())
            ).unless(mIntaker::hasGamePiece)
        );
    }
}
