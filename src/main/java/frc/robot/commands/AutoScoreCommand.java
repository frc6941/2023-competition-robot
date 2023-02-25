package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.ScoringTarget;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;

public class AutoScoreCommand extends SequentialCommandGroup {
    SJTUSwerveMK5Drivebase mDrivebase;
    Intaker mIntaker;
    TargetSelector mTargetSelector;
    ArmAndExtender mSuperstructure;

    Supplier<Pose2d> drivetrainTargetSupplier;
    Supplier<SuperstructureState> superstructureTargetSupplier;
    BooleanSupplier alignedScore;

    private static final Translation3d higherDelta = new Translation3d(0.0, 0.0, 0.05);
    private static final double minDriveX = FieldConstants.Grids.outerX + 0.5;
    private static final double minDriveY = 0.5;
    private static final double maxDriveY = FieldConstants.Community.leftY - 0.5;

    private static final double minExtension = 0.70;
    private static final double lowMaxExtension = 1.0;
    private static final double midMaxExtension = 1.20;
    private static final double highMaxExtension = 1.35;
    

    public AutoScoreCommand(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector, BooleanSupplier confirmation, BooleanSupplier alignedScore) {
        this.mDrivebase = mDrivebase;
        this.mTargetSelector = mTargetSelector;
        this.mSuperstructure = mSuperstructure;
        this.alignedScore = alignedScore;
        this.mIntaker = mIntaker;

        initSuppliers();
        addCommands(
            new RequestSuperstructureStateAutoRetract(mSuperstructure, superstructureTargetSupplier)
            .alongWith(new DriveToPoseCommand(mDrivebase, drivetrainTargetSupplier))
            .andThen(new WaitUntilCommand(confirmation))
            .andThen(new InstantCommand(() -> mIntaker.setIntakerPower(Constants.SUBSYSTEM_INTAKE.OUTTAKING_SLOW_PERCENTAGE)))
            .andThen(new WaitCommand(0.3))
        );
    }

    public void initSuppliers() {
        drivetrainTargetSupplier = () -> getDrivetrainTarget(mDrivebase.getLocalizer().getLatestPose(), mTargetSelector.getScoringDirection());
        superstructureTargetSupplier = () -> getSupertructureStateTarget(
            getDrivetrainTarget(mDrivebase.getLocalizer().getLatestPose(), mTargetSelector.getScoringDirection()),
            mTargetSelector.getScoringDirection()
        );
    }

    public Pose2d getDrivetrainTarget(Pose2d currentPose, Direction direction) {
        Translation3d endEffectorTarget = getEndEffectorTargetPosition();
        double endEffectorZ = endEffectorTarget.getZ();
        Translation2d endEffectorXYPlane = AllianceFlipUtil.apply(endEffectorTarget.toTranslation2d());

        Translation2d deltaTranslation = currentPose.getTranslation().minus(endEffectorXYPlane);
        double maxDistance = getMaxSuperstructureExtension(direction, endEffectorZ);
        double distanceFromNode = Util.clamp(
            deltaTranslation.getNorm(),
            minExtension,
            maxDistance
        );
        
        if(alignedScore.getAsBoolean()) {
            return new Pose2d(
                new Translation2d(
                    minDriveX, endEffectorXYPlane.getY()
                ),
                direction == Direction.FAR ? new Rotation2d() : Rotation2d.fromDegrees(180.0)
            );
        }
        
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

    public SuperstructureState getSupertructureStateTarget(Pose2d currentPose, Direction direction) {
        Translation3d endEffectorTarget = getEndEffectorTargetPosition();
        Translation2d endEffectorXZPlane = new Translation2d(endEffectorTarget.getX(), endEffectorTarget.getZ());
        Translation2d endEffectorXYPlane = endEffectorTarget.toTranslation2d();

        Translation2d deltaTranslation = currentPose.getTranslation().minus(endEffectorXYPlane);
        double deltaDistance = Math.min(deltaTranslation.getNorm(), getMaxSuperstructureExtension(direction, endEffectorXZPlane.getY()));
        SuperstructureState armTarget = SuperstructureKinematics.inverseKinematics2d(
            new Translation2d(
                deltaDistance * (direction == Direction.NEAR ? 1 : -1),
                endEffectorXZPlane.getY()
            )
        );

        return armTarget;
    }

    public Translation3d getEndEffectorTargetPosition() {
        ScoringTarget target = mTargetSelector.getScoringTarget();
        GamePiece gamePiece = mTargetSelector.getTargetGamePiece();

        Translation3d targetEndEffectorPosition;
        switch(target.getScoringRow()){
            case HIGH:
                targetEndEffectorPosition = FieldConstants.Grids.high3dTranslations[target.getPosition(gamePiece)].plus(higherDelta);
                break;
            case MID:
                targetEndEffectorPosition = FieldConstants.Grids.mid3dTranslations[target.getPosition(gamePiece)].plus(higherDelta);
                break;
            case LOW:
                Translation2d temp = FieldConstants.Grids.complexLowTranslations[target.getPosition(gamePiece)];
                targetEndEffectorPosition = new Translation3d(temp.getX(), temp.getY(), 0.0).plus(higherDelta);
                break;
            default:
                targetEndEffectorPosition = null;
                break;
        }

        return targetEndEffectorPosition;
    }

    public double getMaxSuperstructureExtension(Direction direction, double height) {
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
        switch(mTargetSelector.getScoringTarget().getScoringRow()){
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

        return Math.min(armMaxExtension, presetMaxExtension);
    }
}
