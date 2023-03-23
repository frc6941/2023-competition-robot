package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;

import com.ctre.phoenix.GadgeteerUartClient.GadgeteerConnection;
import com.team254.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
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
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.IntArrayToLong;
import frc.robot.utils.GetNearestNumber;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;

public class TargetSelector extends SubsystemBase implements Updatable {
    public static class TargetSelectorPeriodicIO {
        public long[] cursor = new long[] { 1, 5 };
        public long[] target = new long[] { 1, 5 };
        public boolean isCube = false;
        public boolean commuteNear = false;
        public boolean statusHasChanged = false;

        public long loadingTarget = 0;
    }

    public static int[] cubeColumns = new int[] { 1, 4, 7 };
    public static int[] coneColumns = new int[] { 0, 2, 3, 5, 6, 8 };
    
    public TargetSelectorPeriodicIO mPeriodicIO = new TargetSelectorPeriodicIO();

    private NetworkTable targetSelectorTable = NetworkTableInstance.getDefault().getTable("TargetSelector");
    private IntegerArrayPublisher cursorTopic = targetSelectorTable.getIntegerArrayTopic("cursor").publish();
    private IntegerArrayPublisher targetTopic = targetSelectorTable.getIntegerArrayTopic("target").publish();
    private IntegerPublisher loadingTopic = targetSelectorTable.getIntegerTopic("load").publish();
    private StringPublisher targetStringTopic = targetSelectorTable.getStringTopic("targetString").publish();
    private BooleanPublisher isCubeTopic = targetSelectorTable.getBooleanTopic("isCube").publish();
    private BooleanPublisher commuteNear = targetSelectorTable.getBooleanTopic("commuteNear").publish();

    private GamePiece targetGamePiece = GamePiece.CONE;
    private ScoringTarget scoringTarget = new ScoringTarget(SCORING_ROW.HIGH, SCORING_GRID.INNER, SCORING_SIDE.OUTER);
    private LoadingTarget loadingTarget = new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION);

    private Direction scoringDirection = Direction.FAR;
    private Direction commutingDirection = Direction.FAR;
    private Direction loadingDirection = Direction.FAR;

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
        isCubeTopic.setDefault(false);

        loadingTarget = new LoadingTarget((int) mPeriodicIO.loadingTarget);
        int[] transformed = new int[] { 0, 0 };
        transformed[0] = (int) mPeriodicIO.target[0];
        transformed[1] = AllianceFlipUtil.shouldFlip() ? 8 - (int) mPeriodicIO.target[1]: (int) mPeriodicIO.target[1];

        scoringTarget = new ScoringTarget(transformed);
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
        mPeriodicIO.target = IntArrayToLong.apply(scoringTarget.getTargetArray());
        mPeriodicIO.cursor = IntArrayToLong.apply(scoringTarget.getTargetArray());
    }

    public LoadingTarget getLoadingTarget() {
        return this.loadingTarget;
    }

    public void setLoadingTarget(LoadingTarget loadingTarget) {
        this.loadingTarget = loadingTarget;
        mPeriodicIO.loadingTarget = loadingTarget.getLoadingLocation().id;
    }

    public Direction getScoringDirection() {
        return this.scoringDirection;
    }

    public void setScoringDirection(Direction scoringDirection) {
        this.scoringDirection = scoringDirection;
    }

    public void toggleCanCommuteNear() {
        this.mPeriodicIO.commuteNear = !this.mPeriodicIO.commuteNear;
        mPeriodicIO.statusHasChanged = true;
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
        return SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget, loadingDirection, targetGamePiece);
    }

    public SuperstructureState getLoadSuperstructureStateMinExtenderLength() {
        return new SuperstructureState(
            SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget, loadingDirection, targetGamePiece).armAngle,
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
    
    public void setCone() {
        if(mPeriodicIO.isCube) {
            mPeriodicIO.statusHasChanged = true;
        }
        mPeriodicIO.isCube = false;
    }

    public void setCube() {
        if(!mPeriodicIO.isCube) {
            mPeriodicIO.statusHasChanged = true;
        }
        mPeriodicIO.isCube = true;
    }

    public void moveCursor(int rowDelta, int columnDelta) {
        long[] beforeMove = mPeriodicIO.cursor.clone();
        long[] afterMove = clampTargetSelect(new long[] { beforeMove[0] + rowDelta, beforeMove[1] + columnDelta });

        if(afterMove == null) {
            if(mPeriodicIO.isCube && (beforeMove[0] + rowDelta) == 2) {
                // At high row and is restricted, being the cube case, so the target can be moved
                int beforeMoveStartIndex = GetNearestNumber.getIndex((int) beforeMove[1], cubeColumns);
                int maxColumn = cubeColumns.length;
                long afterMoveColumn = (long) (mPeriodicIO.isCube ? cubeColumns : coneColumns)[MathUtil.clamp(beforeMoveStartIndex + columnDelta, 0, maxColumn - 1)];
                mPeriodicIO.cursor = new long[] { 2, afterMoveColumn };
            } else {
                // Does not move cursor if not satisfy the previous condition
            }
        } else if(afterMove[0] == 0) {
            // Handle the low row case
            mPeriodicIO.cursor = afterMove;
        } else {
            // Normal move
            int beforeMoveStartIndex = GetNearestNumber.getIndex((int) beforeMove[1], mPeriodicIO.isCube ? cubeColumns : coneColumns);
            int maxColumn = mPeriodicIO.isCube ? cubeColumns.length : coneColumns.length;
            long afterMoveColumn = (long) (mPeriodicIO.isCube ? cubeColumns : coneColumns)[MathUtil.clamp(beforeMoveStartIndex + columnDelta, 0, maxColumn - 1)];
            mPeriodicIO.cursor = new long[] { afterMove[0], afterMoveColumn };
        }
    }

    public void applyCursorToTarget() {
        mPeriodicIO.target = mPeriodicIO.cursor;
        
        int[] transformed = new int[] { 0, 0 };
        transformed[0] = (int) mPeriodicIO.target[0];
        transformed[1] = AllianceFlipUtil.shouldFlip() ? 8 - (int) mPeriodicIO.target[1]: (int) mPeriodicIO.target[1];
        scoringTarget = new ScoringTarget(transformed);
    }

    private long[] clampTargetSelect(long[] ids) {
        long[] unrestrictedTarget =  new long[] { (long) Util.clamp(ids[0], 0, 2), (long) Util.clamp(ids[1], 0, 8) };
        boolean isCube = unrestrictedTarget[1] == 1 || unrestrictedTarget[1] == 4 || unrestrictedTarget[1] == 7;
        boolean isHigh = unrestrictedTarget[0] == 2;
        boolean notFlip = loadingTarget.getLoadingLocation() != LOADING_LOCATION.SINGLE_SUBSTATION
        && loadingTarget.getLoadingLocation() != LOADING_LOCATION.GROUND_TIPPED;
        if(!mPeriodicIO.commuteNear && !isCube && isHigh && notFlip) {
            // High cone that require commte near, not move cursor
            return null;
        }
        return unrestrictedTarget;
    }

    private long[] regulateCurrent(long[] unrestrictedTarget) {
        boolean isCube = unrestrictedTarget[1] == 1 || unrestrictedTarget[1] == 4 || unrestrictedTarget[1] == 7;
        boolean isHigh = unrestrictedTarget[0] == 2;
        boolean notFlip = loadingTarget.getLoadingLocation() != LOADING_LOCATION.SINGLE_SUBSTATION
        && loadingTarget.getLoadingLocation() != LOADING_LOCATION.GROUND_TIPPED;

        if(unrestrictedTarget[0] == 0) {
            return unrestrictedTarget;
        } else if(mPeriodicIO.isCube) {
            return new long[] { unrestrictedTarget[0], (long) GetNearestNumber.apply((int)unrestrictedTarget[1], cubeColumns) };
        } else {
            // Handle Cone Case
            if(!mPeriodicIO.commuteNear && !isCube && isHigh && notFlip) {
                // High cone that require commte near, regulate
                return new long[] { 1, unrestrictedTarget[1] };
            }
            return new long[] { unrestrictedTarget[0], (long) GetNearestNumber.apply((int)unrestrictedTarget[1], coneColumns) };
        }
        
    }

    private long clampLoadingTarget(double value) {
        if(value >= 4) {
            value -= 4;
        }
        if(value <= -2) {
            value += 4;
        }
        return (long) Util.clamp(value, 0, 3);
    }

    public void moveLoadingTarget(int delta) {
        mPeriodicIO.loadingTarget = clampLoadingTarget(mPeriodicIO.loadingTarget + delta);
    }

    public void updateDirection() {
        if(DriverStation.isAutonomous()) {
            scoringDirection = Direction.NEAR;
            commutingDirection = Direction.FAR;
            loadingDirection = Direction.FAR;
        } else {
            switch(targetGamePiece) {
                case CONE:
                    if(
                        loadingTarget.getLoadingLocation() == LOADING_LOCATION.SINGLE_SUBSTATION
                        || loadingTarget.getLoadingLocation() == LOADING_LOCATION.GROUND_TIPPED
                    ) {
                        scoringDirection = Direction.NEAR;
                        commutingDirection = Direction.FAR;
                        loadingDirection = Direction.FAR;
                    } else if(mPeriodicIO.commuteNear && scoringTarget.getScoringRow() == SCORING_ROW.HIGH) {
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
    }

    public void updateSelect() {
        LoadingTarget temp = new LoadingTarget((int) mPeriodicIO.loadingTarget);
        if(temp.getLoadingLocation() != loadingTarget.getLoadingLocation() || mPeriodicIO.statusHasChanged) {
            loadingTarget = temp;
            mPeriodicIO.target = regulateCurrent(mPeriodicIO.target);
            mPeriodicIO.cursor = regulateCurrent(mPeriodicIO.cursor);
            mPeriodicIO.statusHasChanged = false;
        }

        if(mPeriodicIO.isCube) {
            targetGamePiece = GamePiece.CUBE;
        } else {
            targetGamePiece = GamePiece.CONE;
        }
    }

    public boolean needTurnToScore(Pose2d currentPose) {
        Rotation2d currentRotation = AllianceFlipUtil.apply(currentPose).getRotation();
        Rotation2d targetRotation = scoringDirection == Direction.NEAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0);

        return !(currentRotation.getCos() * targetRotation.getCos() > 0.5);
    }

    @Override
    public synchronized void read(double time, double dt){
        
    }
    
    @Override
    public synchronized void update(double time, double dt){
        updateDirection();
        updateSelect();
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
        isCubeTopic.set(mPeriodicIO.isCube);
        loadingTopic.set((long) mPeriodicIO.loadingTarget);
        commuteNear.set(mPeriodicIO.commuteNear);
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
