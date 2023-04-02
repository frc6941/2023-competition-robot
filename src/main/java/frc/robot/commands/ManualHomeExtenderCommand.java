package frc.robot.commands;

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
        mSuperstructure.setExtenderPercentage(-0.35);
    }

    @Override
    public void end(boolean interrupted) {
        mSuperstructure.homeExtender(Constants.SUBSYSTEM_EXTENDER.HOME_LENGTH);
    }

    @Override
    public boolean isFinished() {
        boolean isReverse = mSuperstructure.mPeriodicIO.extenderVoltage < -0.05;
        boolean isStatic = Math.abs(mSuperstructure.mPeriodicIO.extenderVelocity) < 0.005;
        boolean isStall = mSuperstructure.mPeriodicIO.extenderCurrent > 10.0;
        boolean withinZeroingRegion = Math.abs(mSuperstructure.mPeriodicIO.extenderLength  - 0.90) < 0.25;
        return isReverse && isStatic && isStall && withinZeroingRegion;
    }
}
