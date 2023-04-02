package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.RumbleCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;
import frc.robot.utils.AllianceFlipUtil;

public class TargetSelector extends SubsystemBase implements Updatable {
    public static class TargetSelectorPeriodicIO {
        public boolean isCube = false;
        public boolean commuteNear = false;

        public SCORING_ROW targetRow = SCORING_ROW.MID;
        public LoadingTarget loadTarget = new LoadingTarget(LOADING_LOCATION.GROUND);
    }

    public static int[] cubeColumns = new int[] { 1, 4, 7 };
    public static int[] coneColumns = new int[] { 0, 2, 3, 5, 6, 8 };
    
    public TargetSelectorPeriodicIO mPeriodicIO = new TargetSelectorPeriodicIO();

    private GamePiece targetGamePiece = GamePiece.CONE;

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
    }

    public GamePiece getTargetGamePiece() {
        return this.targetGamePiece;
    }

    public void setTargetGamePiece(GamePiece targetGamePiece) {
        this.targetGamePiece = targetGamePiece;
    }

    public Direction getScoringDirection() {
        return this.scoringDirection;
    }

    public void setScoringDirection(Direction scoringDirection) {
        this.scoringDirection = scoringDirection;
    }

    public void toggleCanCommuteNear() {
        this.mPeriodicIO.commuteNear = !this.mPeriodicIO.commuteNear;
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

    public SuperstructureState getLoadSuperstructureState() {
        return SuperstructureStateBuilder.buildLoadingSupertructureState(mPeriodicIO.loadTarget, loadingDirection, targetGamePiece);
    }

    public SuperstructureState getLoadSuperstructureStateGround(boolean tipped) {
        return SuperstructureStateBuilder.buildLoadingSupertructureState(new LoadingTarget(tipped ? LOADING_LOCATION.GROUND_TIPPED : LOADING_LOCATION.GROUND), loadingDirection, targetGamePiece);
    }

    public SuperstructureState getLoadSuperstructureStateMinExtenderLength() {
        return new SuperstructureState(
            SuperstructureStateBuilder.buildLoadingSupertructureState(mPeriodicIO.loadTarget, loadingDirection, targetGamePiece).armAngle,
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

    public void updateDirection() {
        if(DriverStation.isAutonomous()) {
            scoringDirection = Direction.NEAR;
            commutingDirection = Direction.FAR;
            loadingDirection = Direction.FAR;
        } else {
            switch(targetGamePiece) {
                case CONE:
                    if(mPeriodicIO.commuteNear) {
                        mPeriodicIO.loadTarget = new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION);
                        scoringDirection = Direction.NEAR;
                        commutingDirection = Direction.NEAR;
                        loadingDirection = Direction.NEAR;
                    } else if(mPeriodicIO.loadTarget.getLoadingLocation() == LOADING_LOCATION.SINGLE_SUBSTATION
                    || mPeriodicIO.loadTarget.getLoadingLocation() == LOADING_LOCATION.GROUND_TIPPED) {
                        scoringDirection = Direction.NEAR;
                        commutingDirection = Direction.FAR;
                        loadingDirection = Direction.FAR;
                    } else {
                        scoringDirection = Direction.FAR;
                        commutingDirection = Direction.FAR;
                        loadingDirection = Direction.FAR;
                    }
                    break;
                case CUBE:
                    if(mPeriodicIO.commuteNear) {
                        mPeriodicIO.loadTarget = new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION);
                        scoringDirection = Direction.NEAR;
                        commutingDirection = Direction.NEAR;
                        loadingDirection = Direction.NEAR;
                    } else {
                        scoringDirection = Direction.FAR;
                        commutingDirection = Direction.FAR;
                        loadingDirection = Direction.FAR;
                    }
                    break;
            }
        }
    }

    public boolean needTurnToScore(Pose2d currentPose) {
        Rotation2d currentRotation = AllianceFlipUtil.apply(currentPose).getRotation();
        Rotation2d targetRotation = scoringDirection == Direction.NEAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0);

        return !(currentRotation.getCos() * targetRotation.getCos() > 0.5);
    }

    public void regulateCurrent() {
        if(mPeriodicIO.loadTarget.getLoadingLocation() != LOADING_LOCATION.GROUND_TIPPED
        && targetGamePiece == GamePiece.CONE
        && scoringDirection == Direction.FAR
        && mPeriodicIO.targetRow == SCORING_ROW.HIGH) {
            mPeriodicIO.targetRow = SCORING_ROW.MID;
            System.out.println("Regulate Invalid Target!");
        }
    }

    @Override
    public synchronized void read(double time, double dt){
        
    }
    
    @Override
    public synchronized void update(double time, double dt){
        updateDirection();
        regulateCurrent();
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
        SmartDashboard.putString("Load Location", mPeriodicIO.loadTarget.getLoadingLocation().toString());
        SmartDashboard.putString("Scoring Row", mPeriodicIO.targetRow.toString());
        SmartDashboard.putString("Target Gamepiece", targetGamePiece.toString());

        SmartDashboard.putBoolean("HIGH", mPeriodicIO.targetRow == SCORING_ROW.HIGH);
        SmartDashboard.putBoolean("MID", mPeriodicIO.targetRow == SCORING_ROW.MID);
        SmartDashboard.putBoolean("LOW", mPeriodicIO.targetRow == SCORING_ROW.LOW);

        SmartDashboard.putBoolean("GAMEPIECE", targetGamePiece == GamePiece.CONE);

        SmartDashboard.putBoolean("DOUBLE SUBSTATION", mPeriodicIO.loadTarget.getLoadingLocation() == LOADING_LOCATION.DOUBLE_SUBSTATION);
        SmartDashboard.putBoolean("GROUND TIPPED", mPeriodicIO.loadTarget.getLoadingLocation() == LOADING_LOCATION.GROUND_TIPPED);
        SmartDashboard.putBoolean("GROUND", mPeriodicIO.loadTarget.getLoadingLocation() == LOADING_LOCATION.GROUND);

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

    public void setTargetRow(SCORING_ROW row) {
        if(targetGamePiece == GamePiece.CONE && scoringDirection == Direction.FAR && row == SCORING_ROW.HIGH) {
            new RumbleCommand(ControlBoard.getInstance().getOperatorController(), 1.0, 0.0).withTimeout(0.5).finallyDo(
                (interrupted) -> ControlBoard.getInstance().getOperatorController().setRumble(0.0, 0.0)
            ).schedule();
            System.out.println("Invalid Node Choice!");
        } else {
            mPeriodicIO.targetRow = row;
        }
    }

    public void setLoadingTarget(LOADING_LOCATION location) {
        if(!mPeriodicIO.commuteNear) {
            mPeriodicIO.loadTarget = new LoadingTarget(location);
        }
    }

    public SCORING_ROW getScoringRow() {
        return mPeriodicIO.targetRow;
    }

    public LoadingTarget getLoadingTarget() {
        return mPeriodicIO.loadTarget;
    }
}
