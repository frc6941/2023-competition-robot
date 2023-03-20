package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.Direction;
import frc.robot.states.SuperstructureState;
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
                new RequestExtenderCommand(mSuperstructure, 1.0, 0.02).unless(mIntaker::hasGamePiece),
                new WaitUntilCommand(mIntaker::hasGamePiece),
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
                new RequestExtenderCommand(mSuperstructure, 0.885, 0.02),
                new WaitUntilNoCollision(() -> mDrivebase.getLocalizer().getLatestPose()),
                new RequestSuperstructureStateAutoRetract(mSuperstructure, mTargetSelector::getCommuteSuperstructureState)
            ).unless(mIntaker::hasGamePiece)
        );
    }
}
