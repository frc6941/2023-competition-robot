package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;

import com.team254.lib.util.Util;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

public class TargetSelector extends SubsystemBase implements Updatable {
    public static class TargetSelectorPeriodicIO {
        public long[] cursor = new long[] { 2, 6 };
        public long[] target = new long[] { 2, 6 };
        public long loadingTarget = -1;
    }
    
    public TargetSelectorPeriodicIO mPeriodicIO = new TargetSelectorPeriodicIO();

    private NetworkTable targetSelectorTable = NetworkTableInstance.getDefault().getTable("TargetSelector");
    private IntegerArrayPublisher cursorTopic = targetSelectorTable.getIntegerArrayTopic("cursor").publish();
    private IntegerArrayPublisher targetTopic = targetSelectorTable.getIntegerArrayTopic("target").publish();
    private IntegerPublisher loadingTopic = targetSelectorTable.getIntegerTopic("load").publish();
    private StringPublisher targetStringTopic = targetSelectorTable.getStringTopic("targetString").publish();

    private GamePiece targetGamePiece = GamePiece.CONE;
    private ScoringTarget scoringTarget = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.OUTER);
    private LoadingTarget loadingTarget = new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION);

    private Direction scoringDirection = Direction.NEAR;
    private Direction commutingDirection = Direction.NEAR;
    private Direction loadingDirection = Direction.NEAR;

    private static TargetSelector instance;

    public static TargetSelector getInstance() {
        if (instance == null) {
            instance = new TargetSelector();
        }
        return instance;
    }
    
    private TargetSelector() {
        cursorTopic.setDefault(new long[] { 0, 0 });
        targetTopic.setDefault(new long[] { 0, 0 });
        loadingTopic.setDefault(-1);
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

    public SuperstructureState getCommuteSuperstructureState() {
        return SuperstructureStateBuilder.buildCommutingSuperstructureState(commutingDirection);
    }

    public SuperstructureState getScoreSuperstructureState() {
        return SuperstructureStateBuilder.buildScoringSupertructureState(scoringTarget, scoringDirection, targetGamePiece);
    }

    public SuperstructureState getScoreLowerDelSuperstructureState() {
        return SuperstructureStateBuilder.buildScoringSupertructureStateLowerDelta(scoringTarget, scoringDirection, targetGamePiece);
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

    public void setCursor(long[] cursor) {
        mPeriodicIO.cursor = clampTargetSelect(cursor);
    }

    public void moveCursor(int rowDelta, int columnDelta) {
        mPeriodicIO.cursor = clampTargetSelect(new long[] { mPeriodicIO.cursor[0] + rowDelta, mPeriodicIO.cursor[1] + columnDelta});
    }

    public void applyCursorToTarget() {
        mPeriodicIO.target = mPeriodicIO.cursor;
        
        int[] transformed = new int[] { 0, 0 };
        transformed[0] = (int) mPeriodicIO.target[0];
        transformed[1] = (int) mPeriodicIO.target[1];
        scoringTarget = new ScoringTarget(transformed);

        if(transformed[0] == 0 || transformed[1] == 1 || transformed[1] == 4 || transformed[1] == 7) {
            targetGamePiece = GamePiece.CUBE;
        } else {
            targetGamePiece = GamePiece.CONE;
        }
    }

    private long[] clampTargetSelect(long[] ids) {
        return new long[] { (long) Util.clamp(ids[0], 0, 2), (long) Util.clamp(ids[1], 0, 8) };
    }

    @Override
    public synchronized void read(double time, double dt){
        
    }
    
    @Override
    public synchronized void update(double time, double dt){
        if(this.scoringTarget.getScoringSide() == SCORING_SIDE.MIDDLE || this.scoringTarget.getScoringRow() == SCORING_ROW.LOW) {
            this.targetGamePiece = GamePiece.CUBE;
        } else {
            this.targetGamePiece = GamePiece.CONE;
        }

        if(DriverStation.isAutonomous()) {
            scoringDirection = Direction.NEAR;
            commutingDirection = Direction.FAR;
            loadingDirection = Direction.FAR;
        } else {
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
        }

        switch(loadingTarget.getLoadingLocation()) {
            case DOUBLE_SUBSTATION:
                mPeriodicIO.loadingTarget = 0;
                break;
            case SINGLE_SUBSTATION:
                mPeriodicIO.loadingTarget = 1;
                break;
            case GROUND:
                mPeriodicIO.loadingTarget = 2;
                break;
            default:
                mPeriodicIO.loadingTarget = -1;
                break;
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
        cursorTopic.set(mPeriodicIO.cursor);
        targetTopic.set(mPeriodicIO.target);
        targetStringTopic.set(scoringTarget.toString());
        loadingTopic.set((long) mPeriodicIO.loadingTarget);
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
        read(time, dt);
    }
}
