package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.states.Direction;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class AutoLoad {
    ArmAndExtender mSuperstructure;
    Intaker mIntaker;
    TargetSelector mTargetSelector;
    BooleanSupplier confirmation;

    private Command driveCommand;
    private Command armCommand;

    public static final Pose2d doubleSubstationLeftPose = new Pose2d(
            FieldConstants.StagingLocations.translations[1].getX() - 1.2,
            FieldConstants.StagingLocations.translations[1].getY() + 0.65,
            new Rotation2d());
    public static final Pose2d doubleSubstationRightPose = new Pose2d(
        FieldConstants.StagingLocations.translations[1].getX() - 1.2,
        FieldConstants.StagingLocations.translations[1].getY() - 0.65,
            new Rotation2d());

    public static final double minDriveX = FieldConstants.Grids.outerX + 0.5;
    public static final double minDriveY = 0.5;
    public static final double maxDriveY = FieldConstants.Community.leftY - 0.5;

    public static Map<Object, Command> loadArmCommandMap;
    public static Map<Object, Command> loadDriveCommandMap;

    public AutoLoad(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector, BooleanSupplier confirmation) {
        this.mSuperstructure = mSuperstructure;
        this.mIntaker = mIntaker;
        this.mTargetSelector = mTargetSelector;
        this.confirmation = confirmation;

        loadDriveCommandMap = Map.of(
            LOADING_LOCATION.DOUBLE_SUBSTATION,
                    Commands.either(
                        new DriveSnapRotationCommand(mDrivebase, () -> Rotation2d.fromDegrees(0.0)),
                        new DriveSnapRotationCommand(mDrivebase, () -> Rotation2d.fromDegrees(180.0)),
                        () -> mTargetSelector.getLoadingDirection() == Direction.NEAR
                    ),
            LOADING_LOCATION.SINGLE_SUBSTATION, new DriveToSingleSubstationCommand(mDrivebase),
            LOADING_LOCATION.GROUND, Commands.none()
        );
        loadArmCommandMap = Map.of(
            LOADING_LOCATION.DOUBLE_SUBSTATION, new LoadShelfCommand(mDrivebase, mSuperstructure, mIntaker, mTargetSelector, confirmation),
            LOADING_LOCATION.SINGLE_SUBSTATION, new LoadSingleSubstationCommand(mSuperstructure, mIntaker, mTargetSelector),
            LOADING_LOCATION.GROUND, new LoadGroundCommand(mSuperstructure, mIntaker, mTargetSelector),
            LOADING_LOCATION.GROUND_TIPPED, new LoadGroundCommand(mSuperstructure, mIntaker, mTargetSelector)
        );
        

        driveCommand = Commands.select(loadDriveCommandMap, () -> mTargetSelector.getLoadingTarget().getLoadingLocation());
        armCommand = Commands.select(loadArmCommandMap, () -> mTargetSelector.getLoadingTarget().getLoadingLocation());
    }
    
    public Command getDriveCommand() {
        return driveCommand;
    }

    public Command getArmCommand() {
        return armCommand;
    }
}
