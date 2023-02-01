package frc.robot.coordinators;

import java.util.List;
import java.util.Optional;

import org.frcteam6941.control.DirectionalPose2d;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;
import frc.robot.motion.AStarPathProvider;
import frc.robot.motion.PathProvider;
import frc.robot.states.AssistedPoseBuilder;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget;
import frc.robot.states.ScoringTarget;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;

public class Coordinator implements Updatable {
    public static class PeriodicIO {
        /** INPUTS */
        // Swerve Variables
        public Translation2d inSwerveTranslation = new Translation2d();
        public double inSwerveRotation = 0.0;
        public SWERVE_CARDINAL inSwerveSnapRotation = SWERVE_CARDINAL.NONE;
        public boolean inSwerveBrake = false;
        public double inSwerveFieldHeadingAngle = 0.0;
        public double inSwerveAngularVelocity = 0.0;
        public DirectionalPose2d inSwervePoseAssisted = null;

        public SuperstructureState inCurrentSuperstructureState = new SuperstructureState();
        public boolean inIntakerHasGamePiece = false;

        /** OUTPUTS */
        // Swerve Variables
        public Translation2d outSwerveTranslation = new Translation2d();
        public double outSwerveRotation = 0.0;
        public boolean outSwerveLockHeading = false;
        public double outSwerveHeadingTarget = 0.0;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();
    private final Intaker mIntaker = Intaker.getInstance();
    private final ArmAndExtender mArmAndExtender = ArmAndExtender.getInstance();

    // Core control variabls
    public SuperstructureState coreSuperstructureState;
    public double coreIntakerPower;
    public DirectionalPose2d coreDirectionalPose2d;

    public LoadingTarget loadingTarget = new LoadingTarget(GamePiece.CONE, LOADING_LOCATION.DOUBLE_SUBSTATION_OUTER);
    public ScoringTarget scoringTarget = new ScoringTarget(GamePiece.CONE, SCORING_ROW.MID, SCORING_GRID.OUTER,
            SCORING_SIDE.OUTER);
    public Direction loadDirection = Direction.NEAR;
    public Direction commuteDirection = Direction.FAR;
    public Direction scoreDirection = Direction.NEAR;

    public boolean gotGamePieceRecord = false;
    public Timer scoreFinishedTimer = new Timer();

    // Path planning related
    public PathProvider pathProvider = new AStarPathProvider();
    public Timer pathTrackingStartTimer = new Timer();
    public boolean autoTracking = false;
    public boolean wantAutoTracking = false;

    // State machine related
    public STATE state = STATE.COMMUTING;
    public WANTED_ACTION wantedAction = WANTED_ACTION.COMMUTE;
    public boolean wantedActionChanged = false;

    // Swerve setting related variables
    private boolean swerveSelfLocking = false;
    private Double swerveSelfLockheadingRecord;

    // TODO: Change this to physical button and trigger board after simulation
    private LoggedDashboardChooser<GamePiece> simulatedTargetGamepiece = new LoggedDashboardChooser<GamePiece>(
            "Game Piece");
    private LoggedDashboardChooser<SCORING_GRID> simulatedScoringGridChooser = new LoggedDashboardChooser<SCORING_GRID>(
            "Target Grid");
    private LoggedDashboardChooser<SCORING_SIDE> simulatedScoringSideChooser = new LoggedDashboardChooser<SCORING_SIDE>(
            "Target Side");
    private LoggedDashboardChooser<SCORING_ROW> simulatedScoringRowChooser = new LoggedDashboardChooser<SCORING_ROW>(
            "Target Row");
    private LoggedDashboardChooser<LOADING_LOCATION> simulatedLoadingLocationChooser = new LoggedDashboardChooser<LOADING_LOCATION>(
            "Loading Location");
    private LoggedDashboardChooser<Boolean> simulatedGotGamepiece = new LoggedDashboardChooser<Boolean>(
            "Simulated Game Piece");

    private static Coordinator instance;

    public static Coordinator getInstance() {
        if (instance == null) {
            instance = new Coordinator();
        }
        return instance;
    }

    private Coordinator() {
        List.of(GamePiece.class.getEnumConstants())
                .forEach(x -> simulatedTargetGamepiece.addDefaultOption(x.toString(), x));
        List.of(SCORING_GRID.class.getEnumConstants())
                .forEach(x -> simulatedScoringGridChooser.addDefaultOption(x.toString(), x));
        List.of(SCORING_SIDE.class.getEnumConstants())
                .forEach(x -> simulatedScoringSideChooser.addDefaultOption(x.toString(), x));
        List.of(SCORING_ROW.class.getEnumConstants())
                .forEach(x -> simulatedScoringRowChooser.addDefaultOption(x.toString(), x));
        List.of(LOADING_LOCATION.class.getEnumConstants())
                .forEach(x -> simulatedLoadingLocationChooser.addDefaultOption(x.toString(), x));
        simulatedGotGamepiece.addDefaultOption("No", false);
        simulatedGotGamepiece.addDefaultOption("Yes", true);
    }

    public synchronized void updateDriverAndOperatorCommand() {
        mPeriodicIO.inSwerveTranslation = mControlBoard.getSwerveTranslation();
        mPeriodicIO.inSwerveRotation = mControlBoard.getSwerveRotation();
        mPeriodicIO.inSwerveSnapRotation = mControlBoard.getSwerveSnapRotation();
        mPeriodicIO.inSwerveBrake = mControlBoard.getSwerveBrake();

        if (mControlBoard.zeroGyro()) {
            mSwerve.resetYaw(0.0);
            Pose2d currentPose = mSwerve.getPose();
            mSwerve.getLocalizer().reset(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
            swerveSelfLockheadingRecord = null;
            mSwerve.resetHeadingController();
        }

        if (mControlBoard.getAutoTracking()) {
            wantAutoTracking = !wantAutoTracking;
        }
    }

    /**
     * Update Swerve status.
     * 
     * This method should be called in the end when all the other things are
     * decided.
     */
    public synchronized void updateSwerve() {
        mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
        mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
        if (mPeriodicIO.inSwervePoseAssisted != null) {
            if (mPeriodicIO.inSwervePoseAssisted.isThetaRestricted()) {
                mPeriodicIO.outSwerveLockHeading = true;
                mPeriodicIO.outSwerveHeadingTarget = mPeriodicIO.inSwervePoseAssisted.getRotation().getDegrees();
                swerveSelfLockheadingRecord = null;
            }
        } else if (mPeriodicIO.inSwerveSnapRotation != SWERVE_CARDINAL.NONE) {
            mPeriodicIO.outSwerveLockHeading = true;
            mPeriodicIO.outSwerveHeadingTarget = mPeriodicIO.inSwerveSnapRotation.degrees;
            swerveSelfLockheadingRecord = null;
        } else if (Math.abs(mPeriodicIO.outSwerveRotation) <= 0.03
                && Math.abs(mPeriodicIO.inSwerveAngularVelocity) < 20.0
                && swerveSelfLocking) {
            mPeriodicIO.outSwerveLockHeading = true;
            swerveSelfLockheadingRecord = Optional
                    .ofNullable(swerveSelfLockheadingRecord)
                    .orElse(mPeriodicIO.inSwerveFieldHeadingAngle);
            mPeriodicIO.outSwerveHeadingTarget = swerveSelfLockheadingRecord;
        } else {
            mPeriodicIO.outSwerveLockHeading = false;
            mPeriodicIO.outSwerveHeadingTarget = 0.0;
            swerveSelfLockheadingRecord = null;
        }
    }

    /*
     * Update the direction of loading, commuting and scoring.
     * The guiding principle is to let the driver turn as less as possible.
     */
    public void updateDirections() {
        if (scoringTarget.getScoringRow() == SCORING_ROW.HIGH) {
            loadDirection = Direction.NEAR;
            commuteDirection = Direction.FAR;
            scoreDirection = Direction.NEAR;
        } else if (gotGamePieceRecord) {
            gotGamePieceRecord = false;
            // Load on NEAR side
            if (mPeriodicIO.inCurrentSuperstructureState.armAngle.getCos() > 0.0) {
                loadDirection = Direction.NEAR;
                commuteDirection = Direction.FAR;
                scoreDirection = Direction.NEAR;
            // Load on FAR side
            } else {
                loadDirection = Direction.FAR;
                commuteDirection = Direction.NEAR;
                scoreDirection = Direction.FAR;
            }
        } else {
            if (state == STATE.COMMUTING) {
                if (Rotation2d.fromDegrees(mPeriodicIO.inSwerveFieldHeadingAngle).getCos() > 0.0) {
                    loadDirection = Direction.NEAR;
                } else {
                    loadDirection = Direction.FAR;
                }
            }
        }
    }

    public void updateStates(double dt) {
        if(state != STATE.PREP_SCORING) {
            autoTracking = false;
            wantAutoTracking = false;
            pathTrackingStartTimer.reset();
            pathTrackingStartTimer.stop();
            pathProvider.clear();
        }

        switch (state) {
            case SCORING:
                coreIntakerPower = 0.0;
                coreSuperstructureState = SuperstructureStateBuilder.buildScoringSupertructureState(scoringTarget, scoreDirection);
                coreDirectionalPose2d = AssistedPoseBuilder.buildScoringDirectionalPose2d(scoringTarget, scoreDirection);
                Transform2d delta = coreDirectionalPose2d.minus(mSwerve.getLocalizer().getLatestPose());
                if (Util.epsilonEquals(0, delta.getX(), 0.05) && Util.epsilonEquals(0, delta.getY(), 0.05) && Util.epsilonEquals(0, delta.getRotation().getDegrees(), 3.5)) {
                    coreSuperstructureState = SuperstructureStateBuilder.buildScoringSupertructureStateLowerDelta(scoringTarget, scoreDirection);
                    if (mArmAndExtender.isOnTarget()) {
                        scoreFinishedTimer.start();
                        coreIntakerPower = Constants.SUBSYSTEM_INTAKE.OUTTAKING_PERCENTAGE;
                    }
                }
                break;
            case PREP_SCORING:
                coreIntakerPower = 0.0;
                coreSuperstructureState = SuperstructureStateBuilder.buildScoringSupertructureState(scoringTarget,
                        scoreDirection);
                DirectionalPose2d targetPose = AssistedPoseBuilder.buildScoringDirectionalPose2d(scoringTarget, scoreDirection);
                if(wantAutoTracking && !autoTracking){
                    pathProvider.buildPath(mSwerve.getLocalizer().getLatestPose().getTranslation(), targetPose.getTranslation());
                    pathTrackingStartTimer.reset();
                    pathTrackingStartTimer.stop();
                    autoTracking = true;
                } else if (!wantAutoTracking && autoTracking) {
                    pathTrackingStartTimer.reset();
                    pathTrackingStartTimer.stop();
                    pathProvider.clear();
                    autoTracking = false;
                }

                if(autoTracking) {
                    pathProvider.getPath().ifPresentOrElse(path -> {
                        pathTrackingStartTimer.start();
                        coreDirectionalPose2d = new DirectionalPose2d(
                            new Pose2d(path.getPathPointByDistance(pathTrackingStartTimer.get() * 2.0), targetPose.getRotation()),
                            true, true, true);
                    }, () -> {
                        coreDirectionalPose2d = null;
                    });
                } else {
                    
                }
                break;
            case COMMUTING:
                coreIntakerPower = 0.0;
                coreSuperstructureState = SuperstructureStateBuilder
                        .buildCommutingSuperstructureState(commuteDirection);
                coreDirectionalPose2d = null;
                break;
            case LOADING:
                coreIntakerPower = Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE;
                coreSuperstructureState = SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget,
                        loadDirection);
                coreDirectionalPose2d = AssistedPoseBuilder.buildLoadingDirectionalPose2d(loadingTarget, loadDirection);
                break;
            
            case MANUAL:
                break;
        }
    }

    /**
     * Handle transitions between states.
     */
    public void handleTransitions() {
        if (wantedAction == WANTED_ACTION.MANUAL && state != STATE.MANUAL) {
            setState(STATE.MANUAL);
            return;
        } else {
            switch (state) {
                case PREP_SCORING:
                    if (wantedAction == WANTED_ACTION.SCORE) {
                        setState(STATE.SCORING);
                    } else if (wantedAction == WANTED_ACTION.COMMUTE) {
                        setState(STATE.COMMUTING);
                    } else {
                        System.out.println(
                                "Transition Invalid: Receive Illegal WANTED_ACTION " + wantedAction.toString());
                    }
                    break;
                case SCORING:
                    if (wantedAction == WANTED_ACTION.COMMUTE) {
                        if (scoreFinishedTimer.get() > 0.5) {
                            scoreFinishedTimer.stop();
                            scoreFinishedTimer.reset();
                            setState(STATE.COMMUTING);
                        }
                    } else {
                        System.out.println(
                                "Transition Invalid: Receive Illegal WANTED_ACTION " + wantedAction.toString());
                    }
                    break;
                case COMMUTING:
                    if (wantedAction == WANTED_ACTION.PREP_SCORE) {
                        setState(STATE.PREP_SCORING);
                    } else if (wantedAction == WANTED_ACTION.LOAD) {
                        setState(STATE.LOADING);
                    } else {
                        System.out.println(
                                "Transition Invalid: Receive Illegal WANTED_ACTION " + wantedAction.toString());
                    }
                    break;
                case LOADING:
                    if (wantedAction == WANTED_ACTION.COMMUTE) {
                        setState(STATE.COMMUTING);
                    } else if (wantedAction == WANTED_ACTION.PREP_SCORE) {
                        setState(STATE.PREP_SCORING);
                    } else {
                        System.out.println(
                                "Transition Invalid: Receive Illegal WANTED_ACTION " + wantedAction.toString());
                    }
                    break;
                case MANUAL:
                    if (wantedAction != WANTED_ACTION.MANUAL) {
                        setState(STATE.COMMUTING);
                    } else {
                        coreSuperstructureState = mPeriodicIO.inCurrentSuperstructureState;
                        coreIntakerPower = 0.0;
                        coreDirectionalPose2d = null;
                    }
                    break;
            }
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveAngularVelocity = mSwerve.getLocalizer().getMeasuredVelocity().getRotation().getDegrees();
        mPeriodicIO.inSwervePoseAssisted = mSwerve.getTargetPose();
        if (!mPeriodicIO.inIntakerHasGamePiece && mIntaker.hasGamePiece()) {
            gotGamePieceRecord = true;
        }
        mPeriodicIO.inIntakerHasGamePiece = mIntaker.hasGamePiece();
        mPeriodicIO.inCurrentSuperstructureState = mArmAndExtender.getCurrentSuperstructureState();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (wantedActionChanged) {
            handleTransitions();
            wantedActionChanged = false;
        }
        updateDirections();
        updateStates(dt);
        updateSwerve();
    }

    @Override
    public synchronized void write(double time, double dt) {
        mArmAndExtender.setSuperstructureState(coreSuperstructureState);
        mSwerve.setTargetPose(coreDirectionalPose2d);
        mIntaker.setIntakerPower(coreIntakerPower);

        // Swerve Drive Logic
        if (mPeriodicIO.inSwerveBrake) {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
        } else {
            if (mPeriodicIO.inSwerveTranslation.getNorm() > Constants.CONTROLBOARD.CONTROLLER_DEADBAND
                    || mPeriodicIO.inSwerveRotation > Constants.CONTROLBOARD.CONTROLLER_DEADBAND) {
                mSwerve.getFollower().cancel();
                mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.DRIVE);
            }

            mSwerve.setLockHeading(mPeriodicIO.outSwerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.outSwerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, true, false);
        }

    }

    @Override
    public synchronized void telemetry() {
        Logger.getInstance().recordOutput("Coordinator/State", state.toString());
    }

    @Override
    public synchronized void start() {
        // Auto Generated Method
    }

    @Override
    public synchronized void stop() {
        // Auto Generated Method
    }

    @Override
    public void simulate(double time, double dt) {
        if (mControlBoard.getCommutePressed()) {
            setWantedAction(WANTED_ACTION.COMMUTE);
        }
        if (mControlBoard.getLoadPressed()) {
            setWantedAction(WANTED_ACTION.LOAD);
        }
        if (mControlBoard.getScorePressed()) {
            setWantedAction(WANTED_ACTION.SCORE);
        }
        if (mControlBoard.getPrepScorePressed()) {
            setWantedAction(WANTED_ACTION.PREP_SCORE);
        }

        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveAngularVelocity = mSwerve.getLocalizer().getMeasuredVelocity().getRotation().getDegrees();
        if (!mPeriodicIO.inIntakerHasGamePiece && mIntaker.hasGamePiece()) {
            gotGamePieceRecord = true;
        }
        mPeriodicIO.inCurrentSuperstructureState = mArmAndExtender.getCurrentSuperstructureState();

        mPeriodicIO.inIntakerHasGamePiece = simulatedGotGamepiece.get();
        scoringTarget.setScoringGrid(simulatedScoringGridChooser.get());
        scoringTarget.setScoringRow(simulatedScoringRowChooser.get());
        scoringTarget.setScoringSide(simulatedScoringSideChooser.get());
        scoringTarget.setTargetGamePiece(simulatedTargetGamepiece.get());

        loadingTarget.setTargetGamePiece(simulatedTargetGamepiece.get());
        loadingTarget.setLoadingLocation(simulatedLoadingLocationChooser.get());
    }

    public enum STATE {
        // TODO: Add A* Auto Navigation
        PREP_SCORING,
        SCORING,
        COMMUTING,
        LOADING,
        MANUAL
    }

    public enum WANTED_ACTION {
        PREP_SCORE,
        SCORE,
        COMMUTE,
        LOAD,
        MANUAL
    }

    public STATE getState() {
        return state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public WANTED_ACTION getWantedAction() {
        return this.wantedAction;
    }

    public void setWantedAction(WANTED_ACTION wantedAction) {
        wantedActionChanged = true;
        this.wantedAction = wantedAction;
    }

    public void setTargets(ScoringTarget scoringTarget, LoadingTarget loadingTarget) {
        this.scoringTarget = scoringTarget;
        this.loadingTarget = loadingTarget;
    }
}
