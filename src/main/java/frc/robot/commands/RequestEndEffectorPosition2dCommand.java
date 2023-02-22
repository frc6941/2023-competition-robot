package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.subsystems.ArmAndExtender;

public class RequestEndEffectorPosition2dCommand extends CommandBase {
    ArmAndExtender mSuperstructure;
    Supplier<Translation2d> targetEndEffectorPosition;

    public RequestEndEffectorPosition2dCommand(ArmAndExtender mSuperstructure,
            Supplier<Translation2d> targetEndEffectorPosition) {
        this.mSuperstructure = mSuperstructure;
        this.targetEndEffectorPosition = targetEndEffectorPosition;
    }

    public RequestEndEffectorPosition2dCommand(ArmAndExtender mSuperstructure, Translation2d targetEndEffectorPosition) {
        this.targetEndEffectorPosition = () -> targetEndEffectorPosition;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mSuperstructure.setSuperstructureState(
            SuperstructureKinematics.inverseKinematics2d(targetEndEffectorPosition.get())
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.isOnTarget();
    }
}
