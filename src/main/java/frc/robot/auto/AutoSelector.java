package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.basics.AutoActions;
import frc.robot.states.ScoringTarget;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.PPAutoBuilder;

public class AutoSelector {
    public enum AUTO_START_POSITION {
        INNER,
        OUTER,
        CENTER
    }

    public enum AUTO_ACTION {
        SCORE_PRELOAD,
        JUST_GRAB,
        GRAB_AND_SCORE
    }

    public enum AUTO_BALANCE {
        YES,
        NO
    }

    private final SendableChooser<AUTO_START_POSITION> autoStartPosition = new SendableChooser<>();
    private final SendableChooser<AUTO_ACTION> autoAction = new SendableChooser<>();
    private final SendableChooser<AUTO_BALANCE> autoBalance = new SendableChooser<>();

    private AutoConfiguration autoConfiguration;
    private boolean autoWarning = false;

    private final AutoActions autoActions;
    private Command builtAutoCommand = Commands.none();

    private AutoSelector() {
        autoStartPosition.addOption("Inner (Field Side)", AUTO_START_POSITION.INNER);
        autoStartPosition.addOption("Outer (Wall Side)", AUTO_START_POSITION.OUTER);
        autoStartPosition.setDefaultOption("Middle", AUTO_START_POSITION.CENTER);

        autoAction.setDefaultOption("Score Preload", AUTO_ACTION.SCORE_PRELOAD);
        autoAction.addOption("Score Preload, then Just Brab", AUTO_ACTION.JUST_GRAB);
        autoAction.addOption("Score Preload, then Grab and Score", AUTO_ACTION.GRAB_AND_SCORE);

        autoBalance.setDefaultOption("DO NOT Balance", AUTO_BALANCE.NO);
        autoBalance.addOption("Balance - MAKE SURE THAT YOUR TEAMMATES DO NOT", AUTO_BALANCE.YES);

        autoConfiguration = new AutoConfiguration(
            autoStartPosition.getSelected(),
            autoAction.getSelected(),
            autoBalance.getSelected()
        );

        autoActions = new AutoActions(SJTUSwerveMK5Drivebase.getInstance(), ArmAndExtender.getInstance(),
                Intaker.getInstance(), TargetSelector.getInstance());
    }

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private static AutoSelector instance;

    public AutoConfiguration getChoosedAutoConfiguration() {
        return new AutoConfiguration(
            autoStartPosition.getSelected(),
            autoAction.getSelected(),
            autoBalance.getSelected()
        );
    }

    public Command buildAuto(AutoConfiguration config) {
        Command actionStage;
        Command balanceStage;

        ScoringTarget objective1;
        ScoringTarget objective2;
        ScoringTarget objective3;
        
        // Build Objectives
        switch (config.startPosition) {
            case INNER:
                objective1 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.INNER);
                objective2 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.MIDDLE);
                objective3 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.OUTER);
                break;
            case OUTER:
                objective1 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.OUTER, SCORING_SIDE.OUTER);
                objective2 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.OUTER, SCORING_SIDE.MIDDLE);
                objective3 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.OUTER, SCORING_SIDE.INNER);
                break;
            case CENTER:
                objective1 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.COOPERTITION, SCORING_SIDE.INNER);
                objective2 = null;
                objective3 = null;
                break;
            default:
                objective1 = null;
                objective2 = null;
                objective3 = null;
                break;
        }
        
        HashMap<String, Command> commandMap = autoActions.getCommandMapping(new ScoringTarget[] { objective1, objective2, objective3 });
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(config.toString(), 3.5, 2.2);
        PPAutoBuilder builder = new PPAutoBuilder(SJTUSwerveMK5Drivebase.getInstance(), commandMap);


        actionStage = builder.fullAuto(trajectory);
        balanceStage = config.ifBalance == AUTO_BALANCE.YES ? autoActions.balance(trajectory.getEndState().poseMeters) : autoActions.commute();

        return Commands.sequence(
            actionStage,
            balanceStage
        );
    }

    public void update() {
        SmartDashboard.putData("Auto Start Position", autoStartPosition);
        SmartDashboard.putData("Auto Action", autoAction);
        SmartDashboard.putData("Auto Balance", autoBalance);

        AutoConfiguration currentConfiguration = getChoosedAutoConfiguration();
        if(!currentConfiguration.equals(autoConfiguration)) {
            if(currentConfiguration.startPosition == AUTO_START_POSITION.CENTER) {
                currentConfiguration.action = AUTO_ACTION.SCORE_PRELOAD;
            }
            this.autoConfiguration = currentConfiguration;
            this.builtAutoCommand = buildAuto(autoConfiguration);
        }
    }

    public Command getAutoCommand() {
        return builtAutoCommand;
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }

    public class AutoConfiguration {
        public AUTO_START_POSITION startPosition;
        public AUTO_ACTION action;
        public AUTO_BALANCE ifBalance;

        public AutoConfiguration(AUTO_START_POSITION startPosition, AUTO_ACTION action, AUTO_BALANCE ifBalance) {
            this.startPosition = startPosition;
            this.action = action;
            this.ifBalance = ifBalance;
        }

        @Override
        public boolean equals(Object o) {
            if(o == this) {
                return true;
            }

            if(o instanceof AutoConfiguration) {
                AutoConfiguration temp = (AutoConfiguration) o;
                return temp.startPosition == this.startPosition && temp.action == this.action && temp.ifBalance == this.ifBalance;
            } else {
                return false;
            }
        }

        @Override
        public String toString() {
            return String.format("%s - %s", startPosition.toString(), action.toString());
        }
    }
}
