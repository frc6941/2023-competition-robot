package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class LoadGroundCommand extends SequentialCommandGroup {
    public LoadGroundCommand(ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector,
            BooleanSupplier confirmation) {
        addCommands(
                Commands.sequence(
                        new RequestExtenderCommand(mSuperstructure, () -> {
                            if (Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_NEGATIVE
                                    .inRange(mSuperstructure.getAngle())) {
                                return MathUtil.clamp(mSuperstructure.getLength(), 0.89, 0.90);
                            } else if (Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.DANGEROUS_POSITIVE
                                    .inRange(mSuperstructure.getAngle())) {
                                return MathUtil.clamp(mSuperstructure.getLength(), 0.89, 0.89 + 0.18);
                            } else {
                                return 0.89;
                            }
                        }, 0.05),
                        new RequestArmCommand(mSuperstructure,
                                () -> mTargetSelector.getLoadSuperstructureState().armAngle.getDegrees(), 2.0),
                        Commands.parallel(
                                new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece)),
                                new RequestSuperstructureStateCommand(mSuperstructure,
                                        () -> mTargetSelector.getLoadSuperstructureStateGround(mTargetSelector
                                                .getLoadingTarget()
                                                .getLoadingLocation() == LOADING_LOCATION.GROUND_TIPPED)))
                                .repeatedly().until(mIntaker::hasGamePiece),
                        new InstantCommand(mIntaker::stopIntake)));
    }
}
