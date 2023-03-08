package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class AutoLoad {
    ArmAndExtender mSuperstructure;
    Intaker mIntaker;
    TargetSelector mTargetSelector;
    BooleanSupplier confirmation;
    DoubleSupplier armDelta;

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

    public AutoLoad(ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector, BooleanSupplier confirmation, DoubleSupplier armDelta) {
        this.mSuperstructure = mSuperstructure;
        this.mIntaker = mIntaker;
        this.mTargetSelector = mTargetSelector;
        this.confirmation = confirmation;
        this.armDelta = armDelta;

        driveCommand = Commands.either(
            Commands.none(),
            Commands.either(
                Commands.none(),
                Commands.none(),
                () -> false
            ),
            () -> mTargetSelector.getLoadingTarget().getLoadingLocation() == LOADING_LOCATION.GROUND
        );
        armCommand = Commands.either(
            new LoadGroundCommand(mSuperstructure, mIntaker, mTargetSelector, armDelta),
            new LoadShelfCommand(mSuperstructure, mIntaker, mTargetSelector, confirmation),
            () -> mTargetSelector.getLoadingTarget().getLoadingLocation() == LOADING_LOCATION.GROUND
        );
    }
    
    public Command getDriveCommand() {
        return driveCommand;
    }

    public Command getArmCommand() {
        return armCommand;
    }
}
