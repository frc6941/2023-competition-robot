package frc.robot.controlboard;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomButtonBoard extends CommandGenericHID {
    public CustomButtonBoard(int port) {
        super(port);
    }

    public enum BUTTON{
        LL(1), LM(2), LR(3), ML(4), MM(5), MR(6), UM(7), UL(11), UR(12);

        public final int id;
        BUTTON(int id) {
            this.id = id;
        }
    }

    public Trigger button(BUTTON button) {
        return this.button(button.id);
    }
}
