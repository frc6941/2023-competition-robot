package frc.robot.subsystems;

import org.frcteam6941.led.AddressableLEDWrapper;
import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controlboard.CustomButtonBoard;
import frc.robot.controlboard.CustomButtonBoard.BUTTON;
import frc.robot.states.AssistedPoseBuilder;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;
import frc.robot.utils.Lights;

public class TargetSelector extends SubsystemBase implements Updatable{
    private GamePiece targetGamePiece = GamePiece.CONE;
    private ScoringTarget scoringTarget = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.OUTER);
    private LoadingTarget loadingTarget = new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION_OUTER);

    private Direction scoringDirection = Direction.NEAR;
    private Direction commutingDirection = Direction.NEAR;
    private Direction loadingDirection = Direction.NEAR;

    private CustomButtonBoard mButtonBoard;
    private AddressableLEDWrapper mIndicator;

    private static TargetSelector instance;

    public static TargetSelector getInstance() {
        if (instance == null) {
            instance = new TargetSelector();
        }
        return instance;
    }
    
    private TargetSelector() {
        mIndicator = new AddressableLEDWrapper(9, 10);
        mIndicator.setIntensity(1.0);
        mIndicator.start(0.05);
    }

    public GamePiece getTargetGamePiece() {
        return this.targetGamePiece;
    }

    public void setTargetGamePiece(GamePiece targetGamePiece) {
        this.targetGamePiece = targetGamePiece;
    }

    public ScoringTarget getScoringTarget() {
        return this.scoringTarget;
    }

    public void setScoringTarget(ScoringTarget scoringTarget) {
        this.scoringTarget = scoringTarget;
    }

    public LoadingTarget getLoadingTarget() {
        return this.loadingTarget;
    }

    public void setLoadingTarget(LoadingTarget loadingTarget) {
        this.loadingTarget = loadingTarget;
    }

    public Direction getScoringDirection() {
        return this.scoringDirection;
    }

    public void setScoringDirection(Direction scoringDirection) {
        this.scoringDirection = scoringDirection;
    }

    public Direction getCommutingDirection() {
        return this.commutingDirection;
    }

    public Direction getLoadingDirection() {
        return this.loadingDirection;
    }

    public void bindButtonBoard(CustomButtonBoard buttonBoard) {
        this.mButtonBoard = buttonBoard;
    }

    public SuperstructureState getCommuteSuperstructureState() {
        return SuperstructureStateBuilder.buildCommutingSuperstructureState(commutingDirection);
    }

    public SuperstructureState getScoreSuperstructureState() {
        return SuperstructureStateBuilder.buildScoringSupertructureState(scoringTarget, scoringDirection, targetGamePiece);
    }

    public SuperstructureState getScoreLowerDelSuperstructureState() {
        return SuperstructureStateBuilder.buildScoringSupertructureStateLowerDelta(scoringTarget, scoringDirection, targetGamePiece);
    }
    
    public Pose2d getScorePose2d() {
        return AssistedPoseBuilder.buildScoringPose2d(scoringTarget, scoringDirection, targetGamePiece);
    }

    public SuperstructureState getLoadSuperstructureState() {
        return SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget, loadingDirection);
    }

    public SuperstructureState getLoadSuperstructureStateMinExtenderLength() {
        return new SuperstructureState(
            SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget, loadingDirection).armAngle,
            Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min
        );
    }

    public Pose2d getLoadPose2d() {
        return AssistedPoseBuilder.buildLoadingPose2d(loadingTarget, loadingDirection);
    }

    public SuperstructureState getHairTriggerSuperstructureState() {
        return SuperstructureStateBuilder.buildHairTriggerSuperstructureState();
    }

    public double getIntakerIntakePercentage() {
        switch(targetGamePiece) {
            case CONE:
                return Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE_CONE;
            case CUBE:
                return Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE_CUBE;
            default:
                return 0.0;
        }
    }

    public double getIntakerHoldPercentage() {
        switch(targetGamePiece) {
            case CONE:
                return Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CONE;
            case CUBE:
                return Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CUBE;
            default:
                return 0.0;
        }
    }

    public void updateIndictor() {
        if(DriverStation.isDSAttached()) {
            if(DriverStation.isEnabled()) {
                switch(targetGamePiece) {
                    case CONE:
                        mIndicator.setPattern(Lights.LOAD_CONE);
                        break;
                    case CUBE:
                        mIndicator.setPattern(Lights.LOAD_CUBE);
                        break;
                }
            } else {
                switch(DriverStation.getAlliance()) {
                    case Red:
                        mIndicator.setPattern(Lights.ALLIANCE_RED);
                        break;
                    case Blue:
                        mIndicator.setPattern(Lights.ALLIANCE_BLUE);
                        break;
                    default:
                        mIndicator.setPattern(Lights.CONNECTING);
                }
            }
        } else {
            mIndicator.setPattern(Lights.CONNECTING);
        }
    }


    @Override
    public synchronized void read(double time, double dt){
    }
    
    @Override
    public synchronized void update(double time, double dt){
        switch(targetGamePiece) {
            case CONE:
                if(scoringTarget.getScoringRow() == SCORING_ROW.HIGH) {
                    scoringDirection = Direction.NEAR;
                    commutingDirection = Direction.NEAR;
                    loadingDirection = Direction.NEAR;
                } else {
                    scoringDirection = Direction.FAR;
                    commutingDirection = Direction.FAR;
                    loadingDirection = Direction.FAR;
                }
                break;
            case CUBE:
                scoringDirection = Direction.FAR;
                commutingDirection = Direction.FAR;
                loadingDirection = Direction.FAR;
                break;
        }

        if(mButtonBoard.getRawButton(BUTTON.LM)) {
            setScoringTarget(new ScoringTarget(SCORING_ROW.LOW, SCORING_GRID.INNER, SCORING_SIDE.OUTER));
        }

        if(mButtonBoard.getRawButton(BUTTON.MM)) {
            setScoringTarget(new ScoringTarget(SCORING_ROW.MID, SCORING_GRID.INNER, SCORING_SIDE.OUTER));
        }

        if(mButtonBoard.getRawButton(BUTTON.UM)) {
            setScoringTarget(new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.OUTER));
        }

        if(mButtonBoard.getRawButton(BUTTON.UL)) {
            setTargetGamePiece(GamePiece.CONE);
        }

        if(mButtonBoard.getRawButton(BUTTON.UR)) {
            setTargetGamePiece(GamePiece.CUBE);
        }

        if(mButtonBoard.getRawButton(BUTTON.LL)) {
            setLoadingTarget(new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION_INNER));
        }

        if(mButtonBoard.getRawButton(BUTTON.LR)) {
            setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND));
        }

        updateIndictor();
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
    }
    
    @Override
    public synchronized void start(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void simulate(double time, double dt){
        // Auto Generated Method
    }
}