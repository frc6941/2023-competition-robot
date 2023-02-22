package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class AutoLoadCommand extends ConditionalCommand {
    public AutoLoadCommand(ArmAndExtender mSuperstructure, Intaker mIntaker,
            TargetSelector mTargetSelector, BooleanSupplier confirmation, BooleanSupplier wantManual) {
        super(new AutoGroundLoadCommand(mSuperstructure, mIntaker, mTargetSelector),
                new AutoShelfLoadCommand(mSuperstructure, mIntaker, mTargetSelector, confirmation,
                        wantManual),
                () -> mTargetSelector.getLoadingTarget().getLoadingLocation() == LOADING_LOCATION.GROUND);
    }
}
