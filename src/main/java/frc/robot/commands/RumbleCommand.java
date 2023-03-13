package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.controlboard.CustomXboxController;

public class RumbleCommand extends InstantCommand{
    public RumbleCommand(CustomXboxController controller, double power, double interval) {
        super(() -> controller.setRumble(power, interval));
    }
}
