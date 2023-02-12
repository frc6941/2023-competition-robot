package frc.robot.coordinators;

import java.util.List;

import org.frcteam6941.control.DirectionalPose2d;
import org.frcteam6941.led.AddressableLEDWrapper;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.team254.lib.util.Util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.utils.Lights;

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
        public double inSupertructureManualAngleDelta = 0.0;
        public double inSupertructureManualLengthDelta = 0.0;
        public double inManualIntakerPercentage = 0.0;

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
    private final AddressableLEDWrapper mIndicator = new AddressableLEDWrapper(0, 3);

    // Overall state settings
    public boolean requirePoseAssist = false;
    public boolean autoDirectionDetermine = false;

    // Core control variables
    public SuperstructureState coreSuperstructureState = new SuperstructureState();
    public double coreIntakerPower = 0.0;
    public double coreIntakeHoldPower = Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CONE;
    public DirectionalPose2d coreDirectionalPose2d = null;

    public LoadingTarget loadingTarget = new LoadingTarget(GamePiece.CONE, LOADING_LOCATION.DOUBLE_SUBSTATION_INNER);
    public ScoringTarget scoringTarget = new ScoringTarget(GamePiece.CONE, SCORING_ROW.HIGH, SCORING_GRID.OUTER,
            SCORING_SIDE.OUTER);
    public Direction loadDirection = Direction.NEAR;
    public Direction commuteDirection = Direction.FAR;
    public Direction scoreDirection = Direction.NEAR;

    public boolean gotGamePieceRecord = false;
    public Double loadAutoXDirectionVelocity = null;
    public SlewRateLimiter loadXVelocitySlewRateLimit = new SlewRateLimiter(0.30, -0.60, 0.0);
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
    public boolean inManual = false;

    public SuperstructureState desiredManualSuperstructureState = null;

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
            mSwerve.resetPose(new Pose2d(mSwerve.getPose().getTranslation(),
                    new Rotation2d()));
        }

        if (mControlBoard.getAutoTracking()) {
            wantAutoTracking = !wantAutoTracking;
        }

        if (mControlBoard.getWantManual() && !inManual) {
            inManual = true;
        }
        if (mControlBoard.getExitManual() && inManual) {
            inManual = false;
        }

        if (inManual) {
            setWantedAction(WANTED_ACTION.MANUAL);
            if (mControlBoard.getManualWantAngleIncrease()) {
                mPeriodicIO.inSupertructureManualAngleDelta = Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.ANGLE_CHANGE_DELTA;
            } else if (mControlBoard.getManualWantAngleDecrease()) {
                mPeriodicIO.inSupertructureManualAngleDelta = -Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.ANGLE_CHANGE_DELTA;
            } else {
                mPeriodicIO.inSupertructureManualAngleDelta = 0.0;
            }

            if (mControlBoard.getManualWantLengthIncrease()) {
                mPeriodicIO.inSupertructureManualLengthDelta = Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.LENGTH_CHANGE_DELTA;
            } else if (mControlBoard.getManualWantLengthDecrease()) {
                mPeriodicIO.inSupertructureManualLengthDelta = -Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.LENGTH_CHANGE_DELTA;
            } else {
                mPeriodicIO.inSupertructureManualLengthDelta = 0.0;
            }
            mPeriodicIO.inManualIntakerPercentage = mControlBoard.getManualIntakerPercentage();

        } else {
            mPeriodicIO.inSupertructureManualLengthDelta = 0.0;
            mPeriodicIO.inSupertructureManualAngleDelta = 0.0;
            mPeriodicIO.inManualIntakerPercentage = 0.0;

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
        }

        if (mControlBoard.getSwerveBrake()) {
            mPeriodicIO.inSwerveBrake = true;
        }
    }

    public synchronized void updateRumble(double time) {
        switch (state) {
            case COMMUTING:
                // Robot inverse notice
                double cos = mSwerve.getLocalizer().getLatestPose().getRotation().getCos();
                if ((cos > 0 && scoreDirection == Direction.NEAR) || (cos <= 0 && scoreDirection == Direction.FAR)) {
                    mControlBoard.setDriverRumble(0.5, 0.5);
                } else {
                    mControlBoard.setDriverRumble(0.0, 0.0);
                }
                break;
            case SCORING:
                mControlBoard.setDriverRumble(0.0, 0.0);
                break;
            case PREP_SCORING:
                mControlBoard.setDriverRumble(0.0, 0.0);
                break;
            case LOADING:
                if (mPeriodicIO.inIntakerHasGamePiece) {
                    mControlBoard.setDriverRumble(0.7, 0.0);
                } else {
                    mControlBoard.setDriverRumble(0.0, 0.0);
                }
                break;
            case MANUAL:
                break;
        }
        if (DriverStation.isTeleopEnabled()) {
            mControlBoard.updateRumble(time);
        }
    }

    /**
     * Update Swerve status.
     * 
     * This method should be called in the end when all the other things are
     * decided.
     */
    public synchronized void updateSwerve() {
        if (loadAutoXDirectionVelocity != null && state == STATE.LOADING) {
            mPeriodicIO.outSwerveTranslation = new Translation2d(
                    loadXVelocitySlewRateLimit.calculate(loadAutoXDirectionVelocity), 0.0);
            mPeriodicIO.outSwerveRotation = 0.0;
        } else {
            mPeriodicIO.outSwerveTranslation = mPeriodicIO.inSwerveTranslation;
            mPeriodicIO.outSwerveRotation = mPeriodicIO.inSwerveRotation;
        }
        if (mPeriodicIO.inSwervePoseAssisted != null && mPeriodicIO.inSwervePoseAssisted.isThetaRestricted()) {
            mPeriodicIO.outSwerveLockHeading = false;
            mPeriodicIO.outSwerveHeadingTarget = 0.0;
        } else if (mPeriodicIO.inSwerveSnapRotation != SWERVE_CARDINAL.NONE) {
            mPeriodicIO.outSwerveLockHeading = true;
            mPeriodicIO.outSwerveHeadingTarget = mPeriodicIO.inSwerveSnapRotation.degrees;
        } else {
            mPeriodicIO.outSwerveLockHeading = false;
        }
    }

    /*
     * Update the direction of loading, commuting and scoring.
     * The guiding principle is to let the driver turn as less as possible.
     */
    public void updateDirections() {
        if (scoringTarget.getTargetGamePiece() == GamePiece.CUBE) {
            loadDirection = Direction.NEAR;
            commuteDirection = Direction.FAR;
            if (scoringTarget.getScoringRow() == SCORING_ROW.HIGH) {
                scoreDirection = Direction.NEAR;
            } else {
                scoreDirection = Direction.FAR;
            }
        } else {
            if (scoringTarget.getScoringRow() == SCORING_ROW.HIGH) {
                loadDirection = Direction.NEAR;
                commuteDirection = Direction.NEAR;
                scoreDirection = Direction.NEAR;
            } else {
                loadDirection = Direction.FAR;
                commuteDirection = Direction.NEAR;
                scoreDirection = Direction.NEAR;
            }
        }
    }

    /* Core methods to update set points within states & do automation. */
    public void updateStates() {
        if (scoringTarget.getTargetGamePiece() == GamePiece.CUBE) {
            coreIntakeHoldPower = Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CUBE;
        } else {
            coreIntakeHoldPower = Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CONE;
        }

        if (state != STATE.PREP_SCORING) {
            autoTracking = false;
            wantAutoTracking = false;
            pathTrackingStartTimer.reset();
            pathTrackingStartTimer.stop();
            pathProvider.clear();
        }

        if (state != STATE.MANUAL) {
            desiredManualSuperstructureState = null;
        }

        if (state != STATE.LOADING) {
            loadAutoXDirectionVelocity = null;
            loadXVelocitySlewRateLimit.reset(0.0);
        }

        if (state != STATE.SCORING) {
            mArmAndExtender.setRetractInMotionOverrdie(false);
        }

        switch (state) {
            case SCORING:
                coreIntakerPower = 0.0;
                mArmAndExtender.setRetractInMotionOverrdie(true);
                coreSuperstructureState = SuperstructureStateBuilder
                        .buildScoringSupertructureStateLowerDelta(scoringTarget, scoreDirection);
                // coreDirectionalPose2d =
                // AssistedPoseBuilder.buildScoringDirectionalPose2d(scoringTarget,
                // scoreDirection);
                coreDirectionalPose2d = null;
                if (mArmAndExtender.isOnTarget()) {
                    scoreFinishedTimer.start();
                    coreIntakerPower = Constants.SUBSYSTEM_INTAKE.OUTTAKING_PERCENTAGE;
                    setWantedAction(WANTED_ACTION.COMMUTE);
                }
                break;
            case PREP_SCORING:
                coreIntakerPower = 0.0;
                // DirectionalPose2d targetPose =
                // AssistedPoseBuilder.buildScoringDirectionalPose2d(scoringTarget,
                // // scoreDirection);
                DirectionalPose2d targetPose = new DirectionalPose2d(
                        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-180.0)), true, true, true);
                if (Math.abs(mSwerve.getLocalizer().getLatestPose().getRotation()
                        .minus(targetPose.getRotation()).getDegrees()) < 45.0
                        && targetPose.minus(mSwerve.getLocalizer().getLatestPose()).getTranslation().getNorm() < 1.00) {
                    coreSuperstructureState = SuperstructureStateBuilder.buildScoringSupertructureState(scoringTarget,
                            scoreDirection);
                } else {
                    coreSuperstructureState = SuperstructureStateBuilder.buildCommutingSuperstructureState(commuteDirection);
                }

                if (wantAutoTracking && !autoTracking) {
                    pathTrackingStartTimer.reset();
                    pathTrackingStartTimer.stop();
                    if (pathProvider.buildPath(mSwerve.getLocalizer().getLatestPose().getTranslation(),
                            targetPose.getTranslation(), new Obstacle[] {})) {
                        autoTracking = true;
                    }
                } else if (!wantAutoTracking && autoTracking) {
                    pathTrackingStartTimer.reset();
                    pathTrackingStartTimer.stop();
                    pathProvider.clear();
                    autoTracking = false;
                }

                if (autoTracking) {
                    System.out.println("Testing");
                    pathProvider.getPath().ifPresentOrElse(path -> {
                        pathTrackingStartTimer.start();
                        coreDirectionalPose2d = new DirectionalPose2d(
                                new Pose2d(path.getPathPointByDistance(pathTrackingStartTimer.get() * 1.5),
                                        targetPose.getRotation()),
                                true, true, true);
                    }, () -> {
                        coreDirectionalPose2d = null;
                    });
                } else {
                    coreDirectionalPose2d = null;
                }
                break;
            case COMMUTING:
                coreIntakerPower = 0.0;
                coreSuperstructureState = SuperstructureStateBuilder
                        .buildCommutingSuperstructureState(commuteDirection);
                coreDirectionalPose2d = null;
                break;
            case LOADING:
                if (scoringTarget.getTargetGamePiece() == GamePiece.CUBE) {
                    coreIntakerPower = Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE_CUBE;
                } else {
                    coreIntakerPower = Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE_CONE;
                }
                SuperstructureState tempState = SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget,
                        loadDirection);
                // coreDirectionalPose2d =
                // AssistedPoseBuilder.buildLoadingDirectionalPose2d(loadingTarget,
                // loadDirection);
                coreDirectionalPose2d = new DirectionalPose2d(new Pose2d(4.5, 0.0, Rotation2d.fromDegrees(0.0)), false,
                        true, true);
                if (mPeriodicIO.inIntakerHasGamePiece) {
                    tempState.extenderLength = Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min;
                    tempState.armAngle = loadDirection == Direction.NEAR
                            ? tempState.armAngle.plus(Rotation2d.fromDegrees(10))
                            : tempState.armAngle.plus(Rotation2d.fromDegrees(-10));
                    if (mArmAndExtender.isOnTarget()) {
                        loadAutoXDirectionVelocity = -0.15;
                        if (mPeriodicIO.inSwerveTranslation.getX() < -0.15) {
                            System.out.println("Teleop want commute auto.");
                            setWantedAction(WANTED_ACTION.COMMUTE);
                        }
                    } else {
                        loadAutoXDirectionVelocity = 0.0;
                    }
                } else {
                    if (!Util.epsilonEquals(coreDirectionalPose2d.getX(), mSwerve.getLocalizer().getLatestPose().getX(),
                            0.2) && mArmAndExtender.isOnTarget()) {
                        loadAutoXDirectionVelocity = 0.25;
                    } else {
                        loadAutoXDirectionVelocity = 0.0;
                    }
                }

                if (Math.abs(mSwerve.getLocalizer().getLatestPose().getRotation()
                        .minus(coreDirectionalPose2d.getRotation()).getDegrees()) < 40.0) {
                    coreSuperstructureState = tempState;
                } else {
                    coreSuperstructureState = SuperstructureStateBuilder
                            .buildCommutingSuperstructureState(commuteDirection);
                }
                break;
            case MANUAL:
                if (mArmAndExtender.isHomed()) {
                    if (desiredManualSuperstructureState == null) {
                        // Case of starting with manual mode, then give the desired state a initial
                        // value
                        desiredManualSuperstructureState = mPeriodicIO.inCurrentSuperstructureState;
                    }

                    desiredManualSuperstructureState = new SuperstructureState(
                            Rotation2d.fromDegrees(Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.ARM_RANGE
                                    .clamp(desiredManualSuperstructureState.armAngle.getDegrees()
                                            + mPeriodicIO.inSupertructureManualAngleDelta)),
                            Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE
                                    .clamp(desiredManualSuperstructureState.extenderLength
                                            + mPeriodicIO.inSupertructureManualLengthDelta));

                    coreSuperstructureState = desiredManualSuperstructureState;
                    coreIntakerPower = mPeriodicIO.inManualIntakerPercentage;
                    coreDirectionalPose2d = null;
                }
                break;
        }
    }

    /**
     * Handle transitions between states.
     */
    public void handleTransitions() {
        if (wantedAction == WANTED_ACTION.MANUAL && state != STATE.MANUAL) {
            // Cancel all control, and fix the superstrucutre in the current position
            desiredManualSuperstructureState = mPeriodicIO.inCurrentSuperstructureState;
            coreSuperstructureState = mPeriodicIO.inCurrentSuperstructureState;
            coreDirectionalPose2d = null;
            coreIntakerPower = 0.0;
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
                        if (scoreFinishedTimer.get() > 1.0) {
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
                    }
                    break;
            }
        }
    }

    public void updateIndicator() {
        if (DriverStation.isEnabled()) {
            switch (state) {
                case PREP_SCORING:
                case SCORING:
                    switch (scoringTarget.getTargetGamePiece()) {
                        case CONE:
                            mIndicator.setPattern(Lights.SCORE_CONE);
                            break;
                        case CUBE:
                            mIndicator.setPattern(Lights.SCORE_CUBE);
                            break;
                    }
                    break;
                case COMMUTING:
                    switch (loadingTarget.getTargetGamePiece()) {
                        case CONE:
                            mIndicator.setPattern(Lights.COMMUTE_CONE);
                            break;
                        case CUBE:
                            mIndicator.setPattern(Lights.COMMUTE_CUBE);
                            break;
                    }
                    break;
                case LOADING:
                    switch (loadingTarget.getTargetGamePiece()) {
                        case CONE:
                            mIndicator.setPattern(Lights.LOAD_CONE);
                            break;

                        case CUBE:
                            mIndicator.setPattern(Lights.LOAD_CUBE);
                            break;
                    }

                    break;
                case MANUAL:
                    mIndicator.setPattern(Lights.MANUAL);
                    break;
            }
        } else {
            switch (DriverStation.getAlliance()) {
                case Red:
                    mIndicator.setPattern(Lights.ALLIANCE_RED);
                    break;
                case Blue:
                    mIndicator.setPattern(Lights.ALLIANCE_BLUE);
                    break;
                default:
                    mIndicator.setPattern(Lights.CONNECTING);
                    break;
            }
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveAngularVelocity = mSwerve.getLocalizer().getMeasuredVelocity().getRotation().getDegrees();
        mPeriodicIO.inSwervePoseAssisted = mSwerve.getTargetPose();
        mPeriodicIO.inIntakerHasGamePiece = mIntaker.hasGamePiece();
        mPeriodicIO.inCurrentSuperstructureState = mArmAndExtender.getCurrentSuperstructureState();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (wantedActionChanged) {
            handleTransitions();
            wantedActionChanged = false;
        }
        if (autoDirectionDetermine) {
            updateDirections();
        }
        updateStates();
        updateIndicator();
        updateRumble(time);

        updateSwerve();
    }

    @Override
    public synchronized void write(double time, double dt) {
        mArmAndExtender.setSuperstructureState(coreSuperstructureState);
        mSwerve.setTargetPose(coreDirectionalPose2d);
        mIntaker.setIntakerPower(coreIntakerPower);
        mIntaker.setHoldPower(coreIntakeHoldPower);

        // Swerve Drive Logic
        if (mPeriodicIO.inSwerveBrake) {
            mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.BRAKE);
        } else {
            if (mPeriodicIO.inSwerveTranslation.getNorm() > Constants.CONTROLBOARD.CONTROLLER_DEADBAND
                    || mPeriodicIO.inSwerveRotation > Constants.CONTROLBOARD.CONTROLLER_DEADBAND) {
                mSwerve.getFollower().cancel();
                mSwerve.setState(SJTUSwerveMK5Drivebase.STATE.DRIVE);
            }

            if (coreDirectionalPose2d != null && coreDirectionalPose2d.isThetaRestricted()) {
                mPeriodicIO.outSwerveHeadingTarget = coreDirectionalPose2d.getRotation().getDegrees();
                mPeriodicIO.outSwerveLockHeading = true;
            }
            mSwerve.setLockHeading(mPeriodicIO.outSwerveLockHeading);
            mSwerve.setHeadingTarget(mPeriodicIO.outSwerveHeadingTarget);
            mSwerve.drive(mPeriodicIO.outSwerveTranslation, mPeriodicIO.outSwerveRotation, true, true);
        }

    }

    @Override
    public synchronized void telemetry() {
        Logger.getInstance().recordOutput("Coordinator/State", state.toString());
    }

    @Override
    public synchronized void start() {
        desiredManualSuperstructureState = null;
        setState(STATE.COMMUTING);
    }

    @Override
    public synchronized void stop() {
        // Auto Generated Method
    }

    @Override
    public void simulate(double time, double dt) {
        if (mControlBoard.getWantManual() && !inManual) {
            inManual = true;
        }
        if (mControlBoard.getExitManual() && inManual) {
            inManual = false;
        }

        if (inManual) {
            setWantedAction(WANTED_ACTION.MANUAL);
            if (mControlBoard.getManualWantAngleIncrease()) {
                mPeriodicIO.inSupertructureManualAngleDelta = Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.ANGLE_CHANGE_DELTA;
            } else if (mControlBoard.getManualWantAngleDecrease()) {
                mPeriodicIO.inSupertructureManualAngleDelta = -Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.ANGLE_CHANGE_DELTA;
            } else {
                mPeriodicIO.inSupertructureManualAngleDelta = 0.0;
            }

            if (mControlBoard.getManualWantLengthIncrease()) {
                mPeriodicIO.inSupertructureManualLengthDelta = Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.LENGTH_CHANGE_DELTA;
            } else if (mControlBoard.getManualWantLengthDecrease()) {
                mPeriodicIO.inSupertructureManualLengthDelta = -Constants.SUBSYSTEM_SUPERSTRUCTURE.MANUAL_DELTA.LENGTH_CHANGE_DELTA;
            } else {
                mPeriodicIO.inSupertructureManualLengthDelta = 0.0;
            }
            mPeriodicIO.inManualIntakerPercentage = mControlBoard.getManualIntakerPercentage();

        } else {
            mPeriodicIO.inSupertructureManualLengthDelta = 0.0;
            mPeriodicIO.inSupertructureManualAngleDelta = 0.0;
            mPeriodicIO.inManualIntakerPercentage = 0.0;

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
        }

        if (mControlBoard.getSwerveBrake()) {
            mPeriodicIO.inSwerveBrake = true;
        }

        mPeriodicIO.inSwerveFieldHeadingAngle = mSwerve.getYaw();
        mPeriodicIO.inSwerveAngularVelocity = mSwerve.getLocalizer().getMeasuredVelocity().getRotation().getDegrees();
        mPeriodicIO.inIntakerHasGamePiece = false;
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
