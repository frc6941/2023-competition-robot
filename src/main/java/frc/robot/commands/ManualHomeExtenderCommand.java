package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmAndExtender;

public class ManualHomeExtenderCommand extends CommandBase{
    ArmAndExtender mSuperstructure;

    public ManualHomeExtenderCommand(ArmAndExtender mSuperstructure) {
        this.mSuperstructure = mSuperstructure;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mSuperstructure.setExtenderPercentage(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        mSuperstructure.homeExtender(Constants.SUBSYSTEM_EXTENDER.HOME_LENGTH);
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.mPeriodicIO.extenderCurrent > 10.0;
    }
}
