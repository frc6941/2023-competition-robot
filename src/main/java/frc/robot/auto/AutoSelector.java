package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.auto.basics.AutoActions;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.states.ScoringTarget;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;

public class AutoSelector {
    public enum AUTO_START_POSITION {
        INNER,
        OUTER,
        CENTER
    }

    public enum AUTO_ACTION {
        DO_NOTHING,
        MOBILITY,
        SCORE_PRELOAD,
        TWO_GAMEPIECE,
        LINK
    }

    public enum AUTO_BALANCE {
        YES,
        NO
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
    }

    private final SendableChooser<AUTO_START_POSITION> autoStartPosition = new SendableChooser<>();
    private final SendableChooser<AUTO_ACTION> autoAction = new SendableChooser<>();
    private final SendableChooser<AUTO_BALANCE> autoBalance = new SendableChooser<>();

    private AutoConfiguration autoConfiguration;
    private boolean autoWarning = false;

    private final AutoActions autoBuilder;
    private Command builtAutoCommand = Commands.none();

    private AutoSelector() {
        autoStartPosition.addOption("Inner (Field Side)", AUTO_START_POSITION.INNER);
        autoStartPosition.addOption("Outer (Wall Side)", AUTO_START_POSITION.OUTER);
        autoStartPosition.setDefaultOption("Middle", AUTO_START_POSITION.CENTER);

        autoAction.setDefaultOption("Do Nothing", AUTO_ACTION.DO_NOTHING);
        autoAction.addOption("Mobility", AUTO_ACTION.DO_NOTHING);
        autoAction.addOption("Score Preload", AUTO_ACTION.SCORE_PRELOAD);
        autoAction.addOption("Score Preload, then Get Cube", AUTO_ACTION.TWO_GAMEPIECE);
        autoAction.addOption("Score Link", AUTO_ACTION.LINK);

        autoBalance.setDefaultOption("DO NOT Balance", AUTO_BALANCE.NO);
        autoBalance.addOption("Balance - MAKE SURE THAT YOUR TEAMMATES DO NOT", AUTO_BALANCE.YES);

        autoConfiguration = new AutoConfiguration(
                autoStartPosition.getSelected(), autoAction.getSelected(), autoBalance.getSelected());

        autoBuilder = new AutoActions(SJTUSwerveMK5Drivebase.getInstance(), ArmAndExtender.getInstance(),
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
            autoStartPosition.getSelected(), autoAction.getSelected(), autoBalance.getSelected());
    }

    public Command buildAuto(AutoConfiguration config) {
        Command resetStage = Commands.none();
        Command actionStage;
        Command balanceStage;

        ScoringTarget objective1;
        ScoringTarget objective2;
        ScoringTarget objective3;
        Translation2d intakeTarget1;
        Translation2d intakeTarget2;

        switch (config.startPosition) {
            case INNER:
                objective1 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.INNER);
                objective2 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.MIDDLE);
                objective3 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.OUTER);
                intakeTarget1 = FieldConstants.StagingLocations.translations[3];
                intakeTarget2 = FieldConstants.StagingLocations.translations[2];
                break;
            case OUTER:
                objective1 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.OUTER, SCORING_SIDE.OUTER);
                objective2 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.OUTER, SCORING_SIDE.MIDDLE);
                objective3 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.OUTER, SCORING_SIDE.INNER);
                intakeTarget1 = FieldConstants.StagingLocations.translations[0];
                intakeTarget2 = FieldConstants.StagingLocations.translations[1];
                break;
            case CENTER:
                objective1 = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.COOPERTITION, SCORING_SIDE.MIDDLE);
                objective2 = null;
                objective3 = null;
                intakeTarget1 = null;
                intakeTarget2 = null;
                break;
            default:
                objective1 = null;
                objective2 = null;
                objective3 = null;
                intakeTarget1 = null;
                intakeTarget2 = null;
                break;
        }

        boolean isLeft = config.startPosition == AUTO_START_POSITION.INNER;

        if(config.startPosition != AUTO_START_POSITION.CENTER) {
            switch (config.action) {
                case DO_NOTHING:
                case MOBILITY:
                    actionStage = Commands.none();
                    break;
                case SCORE_PRELOAD:
                    actionStage = autoBuilder.scorePreload(objective1).andThen(autoBuilder.commute());
                    break;
                case TWO_GAMEPIECE:
                    actionStage = Commands.sequence(
                            autoBuilder.scorePreload(objective1),
                            autoBuilder.intakeAndScore(isLeft, objective2, intakeTarget1),
                            autoBuilder.commute());
                    break;
                case LINK:
                    actionStage = Commands.sequence(
                            autoBuilder.scorePreload(objective1),
                            autoBuilder.intakeAndScore(isLeft, objective2, intakeTarget1),
                            autoBuilder.intakeAndScore(isLeft, objective3, intakeTarget2),
                            autoBuilder.commute());
                    break;
                default:
                    actionStage = Commands.none();
                    break;
            }
        } else {
            switch (config.action) {
                case DO_NOTHING:
                case MOBILITY:
                    actionStage = Commands.none();
                    break;
                case SCORE_PRELOAD:
                    actionStage = autoBuilder.scorePreload(objective1);
                    break;
                case TWO_GAMEPIECE:
                case LINK:
                default:
                    actionStage = Commands.none();
                    break;
            }
        }

        switch(config.ifBalance) {
            case YES:
                balanceStage = autoBuilder.balance(isLeft, 0.0);
                break;
            case NO:
            default:
                balanceStage = Commands.none();
        }

        return Commands.sequence(
                resetStage,
                actionStage,
                balanceStage);
    }

    public void update() {
        AutoConfiguration currentConfiguration = getChoosedAutoConfiguration();
        if(!currentConfiguration.equals(autoConfiguration)) {
            this.autoConfiguration = currentConfiguration;
            this.builtAutoCommand = buildAuto(autoConfiguration);
        }

        SmartDashboard.putData("Auto Start Position", autoStartPosition);
        SmartDashboard.putData("Auto Action", autoAction);
        SmartDashboard.putData("Auto Balance", autoBalance);
    }

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Grab Cube", new PathConstraints(2.0, 2.0));

    public Command getAutoCommand() {
        // trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        // SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
        // return Commands.runOnce(() -> mDrivebase.resetPose(trajectory.getInitialHolonomicPose())).andThen(new FollowTrajectoryWithEvents(mDrivebase, trajectory, new HashMap<>()));
        return 
        Commands.runOnce(() -> SJTUSwerveMK5Drivebase.getInstance().resetPose(AllianceFlipUtil.apply(trajectory.getInitialHolonomicPose())))
        // .andThen(autoBuilder.scorePreload(new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.INNER)))
        // .andThen(autoBuilder.configGroundIntake())
        .andThen(
            new FollowTrajectory(
                SJTUSwerveMK5Drivebase.getInstance(),
                trajectory
            )
        );
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }
}
