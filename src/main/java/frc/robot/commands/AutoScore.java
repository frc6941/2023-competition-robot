package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.ScoringTarget;
import frc.robot.states.SuperstructureState;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;

public class AutoScore {
    SJTUSwerveMK5Drivebase mDrivebase;
    Intaker mIntaker;
    TargetSelector mTargetSelector;
    ArmAndExtender mSuperstructure;

    Supplier<Pose2d> drivetrainTargetSupplier;
    Supplier<SuperstructureState> superstructureTargetSupplier;
    Supplier<SuperstructureState> superstructureTargetLoweredSupplier;
    BooleanSupplier alignedScore;

    private static final Translation3d higherDeltaCone = new Translation3d(-0.14, 0.0, 0.25);
    private static final Translation3d midDeltaCone = new Translation3d(-0.14, 0.0, 0.32);

    private static final Translation3d higherDeltaCube = new Translation3d(0.05, 0.0, 0.56);
    private static final Translation3d midDeltaCube = new Translation3d(0.15, 0.0, 0.4);
    private static final Translation3d lowerDeltaCube = new Translation3d(-0.40, 0.0, 0.25);
    private static final double minDriveX = FieldConstants.Grids.outerX + 0.42;
    private static final double minDriveY = 0.5;
    private static final double maxDriveY = FieldConstants.Community.leftY - 0.5;

    private static final double minExtension = 0.60;
    private static final double lowMaxExtension = 0.60;
    private static final double midMaxExtension = 1.20;
    private static final double highMaxExtension = 1.60;

    private Command driveCommand;
    private Command armCommand;
    

    public AutoScore(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector, BooleanSupplier confirmation, BooleanSupplier forceExtend, BooleanSupplier alignedScore) {
        this.mDrivebase = mDrivebase;
        this.mTargetSelector = mTargetSelector;
        this.mSuperstructure = mSuperstructure;
        this.alignedScore = alignedScore;
        this.mIntaker = mIntaker;

        initSuppliers();

        driveCommand = new DriveToPoseCommand(mDrivebase, drivetrainTargetSupplier).andThen(new InstantCommand(() -> mDrivebase.stopMovement()));
        armCommand =
        new ConditionalCommand(
            new RequestSuperstructureStateCommand(mSuperstructure, superstructureTargetSupplier),

            new RequestExtenderCommand(mSuperstructure, 0.89, 0.05)
            .andThen(new RequestArmCommand(mSuperstructure, () -> superstructureTargetSupplier.get().armAngle.getDegrees(), 5.0)),

            () -> {
                Pose2d pose = AllianceFlipUtil.apply(mDrivebase.getLocalizer().getLatestPose());
                boolean onTranslation = pose.getTranslation().minus(drivetrainTargetSupplier.get().getTranslation()).getNorm() < 0.60;
                boolean onRotation = Math.abs(pose.getRotation().minus(drivetrainTargetSupplier.get().getRotation()).getDegrees()) < 30.0;
                boolean armOnTarget = Util.epsilonEquals(superstructureTargetSupplier.get().armAngle.getDegrees(), mSuperstructure.getCurrentSuperstructureState().armAngle.getDegrees(), 5.0);
                boolean override = forceExtend.getAsBoolean();

                return (onTranslation && onRotation && armOnTarget) || override;
            }
        ).repeatedly().until(confirmation)
        // new RequestSuperstructureStateAutoRetract(mSuperstructure, superstructureTargetSupplier).andThen(
        //     new WaitUntilCommand(confirmation)
        // )
        .andThen(
            new RequestSuperstructureStateCommand(mSuperstructure, superstructureTargetLoweredSupplier)
            .andThen(new WaitCommand(0.5))
            .unless(() -> mTargetSelector.getTargetGamePiece() == GamePiece.CUBE)
        )
        .andThen(new PrintCommand("Ejecting Gamepiece"))
        .andThen(new InstantCommand(() -> mIntaker.runOuttake(mTargetSelector::getTargetGamePiece)))
        .andThen(new WaitCommand(0.3))
        .andThen(new WaitUntilNoCollision(() -> mDrivebase.getLocalizer().getLatestPose()));
    }

    public void initSuppliers() {
        drivetrainTargetSupplier = () -> getDrivetrainTarget(mDrivebase.getLocalizer().getLatestPose(), mTargetSelector.getScoringTarget(), mTargetSelector.getTargetGamePiece(), mTargetSelector.getScoringDirection(), alignedScore.getAsBoolean());
        superstructureTargetSupplier = () -> getSupertructureStateTarget(
            getDrivetrainTarget(mDrivebase.getLocalizer().getLatestPose(), mTargetSelector.getScoringTarget(), mTargetSelector.getTargetGamePiece(), mTargetSelector.getScoringDirection(), alignedScore.getAsBoolean()),
            mTargetSelector.getScoringTarget(),
            mTargetSelector.getTargetGamePiece(),
            mTargetSelector.getScoringDirection(),
            0.0
        );
        superstructureTargetLoweredSupplier = () -> getSupertructureStateTarget(
            getDrivetrainTarget(mDrivebase.getLocalizer().getLatestPose(), mTargetSelector.getScoringTarget(), mTargetSelector.getTargetGamePiece(), mTargetSelector.getScoringDirection(), alignedScore.getAsBoolean()),
            mTargetSelector.getScoringTarget(),
            mTargetSelector.getTargetGamePiece(),
            mTargetSelector.getScoringDirection(),
            0.32
        );
    }

    public static Pose2d getDrivetrainTarget(Pose2d currentPose, ScoringTarget target, GamePiece gamepiece, Direction direction, boolean alignedScore) {
        Translation3d endEffectorTarget = getEndEffectorTargetPosition(target, gamepiece);
        double endEffectorZ = endEffectorTarget.getZ();
        Translation2d endEffectorXYPlane = endEffectorTarget.toTranslation2d();

        Translation2d deltaTranslation = AllianceFlipUtil.apply(currentPose).getTranslation().minus(endEffectorXYPlane);
        double maxDistance = getMaxSuperstructureExtension(target, direction, endEffectorZ);
        
        if(alignedScore) {
            return new Pose2d(
                new Translation2d(
                    Math.max(minDriveX, endEffectorXYPlane.getX() + maxDistance), endEffectorXYPlane.getY()
                ),
                direction == Direction.FAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0)
            );
        }

        double distanceFromNode = Util.clamp(
            deltaTranslation.getNorm(),
            minExtension,
            maxDistance
        );
        
        Rotation2d angleFromNode = deltaTranslation.getAngle();
        Rotation2d maxRotation = new Rotation2d(Math.acos((minDriveX - endEffectorXYPlane.getX()) / maxDistance));
        if (angleFromNode.getRadians() > maxRotation.getRadians()) {
            angleFromNode = maxRotation;
        }
        if (angleFromNode.getRadians() < -maxRotation.getRadians()) {
            angleFromNode = maxRotation.unaryMinus();
        }

        double minDistanceAtAngle = (minDriveX - endEffectorXYPlane.getX()) / angleFromNode.getCos();
        if (distanceFromNode < minDistanceAtAngle) {
            distanceFromNode = minDistanceAtAngle;
        }

        Translation2d unrestrictedTranslation = new Translation2d(
            distanceFromNode,
            angleFromNode
        ).plus(endEffectorXYPlane);

        Translation2d restrictedTranslation = new Translation2d(
            unrestrictedTranslation.getX(),
            Util.clamp(unrestrictedTranslation.getY(), minDriveY, maxDriveY)
        );

        Rotation2d skewAngle = restrictedTranslation.minus(endEffectorXYPlane).getAngle();
        Rotation2d delta = direction == Direction.FAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0);

        Pose2d driveTarget = new Pose2d(
            restrictedTranslation,
            skewAngle.plus(delta)
        );
        
        return driveTarget;
    }

    public static SuperstructureState getSupertructureStateTarget(Pose2d currentPose, ScoringTarget target, GamePiece gamepiece, Direction direction, double lower) {
        Translation3d endEffectorTarget = getEndEffectorTargetPosition(target, gamepiece);
        Translation2d endEffectorXZPlane = new Translation2d(endEffectorTarget.getX(), endEffectorTarget.getZ());
        Translation2d endEffectorXYPlane = endEffectorTarget.toTranslation2d();

        Translation2d deltaTranslation = currentPose.getTranslation().minus(endEffectorXYPlane);
        double deltaDistance = Math.min(deltaTranslation.getNorm(), getMaxSuperstructureExtension(target, direction, endEffectorXZPlane.getY()));
        SuperstructureState armTarget = SuperstructureKinematics.inverseKinematics2d(
            new Translation2d(
                deltaDistance * (direction == Direction.NEAR ? 1 : -1),
                endEffectorXZPlane.getY() - lower
            )
        );

        return armTarget;
    }

    public static Translation3d getEndEffectorTargetPosition(ScoringTarget target, GamePiece targetGamepiece) {
        Translation3d targetEndEffectorPosition;
        Translation3d delta;
        if(targetGamepiece == GamePiece.CUBE) {
            if (target.getScoringRow() == SCORING_ROW.HIGH){
                delta = higherDeltaCube;
            } else if(target.getScoringRow() == SCORING_ROW.LOW) {
                delta = lowerDeltaCube;
            } else {
                delta = midDeltaCube;
            }
        } else {
            if(target.getScoringRow() == SCORING_ROW.HIGH) {
                delta = higherDeltaCone;
            } else {
                delta = midDeltaCone;
            }
        }

        switch(target.getScoringRow()){
            case HIGH:
                targetEndEffectorPosition = FieldConstants.Grids.high3dTranslations[target.getPosition()].plus(delta);
                break;
            case MID:
                targetEndEffectorPosition = FieldConstants.Grids.mid3dTranslations[target.getPosition()].plus(delta);
                break;
            case LOW:
                Translation2d temp = FieldConstants.Grids.complexLowTranslations[target.getPosition()];
                targetEndEffectorPosition = new Translation3d(temp.getX(), temp.getY(), 0.0).plus(delta);
                break;
            default:
                targetEndEffectorPosition = null;
                break;
        }

        return targetEndEffectorPosition;
    }

    public static double getMaxSuperstructureExtension(ScoringTarget target, Direction direction, double height) {
        Rotation2d armAngleInitial = direction == Direction.NEAR ? Rotation2d.fromDegrees(-90.0) : Rotation2d.fromDegrees(270.0);

        double armMaxExtension = Math.abs(SuperstructureKinematics.forwardKinematics2d(
            new SuperstructureState(
                Rotation2d.fromDegrees(
                    Units.radiansToDegrees(
                        Math.acos(
                            (Constants.SUBSYSTEM_SUPERSTRUCTURE.STRUCTURE.HIGH_PIVOT_2D_LOCATION.getY()  - height
                            / Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.max)))
                    ).plus(armAngleInitial),
                Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.max
            )
        ).getX());

        double presetMaxExtension;
        switch(target.getScoringRow()){
            case LOW:
                presetMaxExtension = lowMaxExtension;
                break;
            case MID:
                presetMaxExtension = midMaxExtension;
                break;
            case HIGH:
                presetMaxExtension = highMaxExtension;
                break;
            default:
                presetMaxExtension = lowMaxExtension;
                break;
        }

        return Math.min(presetMaxExtension, armMaxExtension);
    }

    public Command getDriveCommand() {
        return driveCommand;
    }

    public Command getArmCommand() {
        return armCommand;
    }

    public BooleanSupplier isArmReady() {
        return () -> Math.abs(superstructureTargetSupplier.get().armAngle.minus(mSuperstructure.getCurrentSuperstructureState().armAngle).getDegrees()) <= 5.0;
    }

    public Supplier<Pose2d> getDrivetrainTargetSupplier() {
        return drivetrainTargetSupplier;
    }

    public Supplier<SuperstructureState> getSuperstructureTargetSupplier() {
        return superstructureTargetSupplier;
    }

    public Supplier<SuperstructureState> getSuperstructureTargetSupplierLower() {
        return superstructureTargetLoweredSupplier;
    }
}
