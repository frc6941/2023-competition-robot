// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all
 * translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost
 * point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {
    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color.
     */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /**
     * Flips an x coordinate to the correct side of the field based on the current
     * alliance color.
     */
    public static double apply(double xCoordinate) {
        if (shouldFlip()) {
            return FieldConstants.fieldLength - xCoordinate;
        } else {
            return xCoordinate;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance
     * color.
     */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                    FieldConstants.fieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the
     * current alliance color.
     */
    public static Trajectory.State apply(Trajectory.State state) {
        if (shouldFlip()) {
            return new Trajectory.State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                            FieldConstants.fieldLength - state.poseMeters.getX(),
                            state.poseMeters.getY(),
                            new Rotation2d(
                                    -state.poseMeters.getRotation().getCos(),
                                    state.poseMeters.getRotation().getSin())),
                    -state.curvatureRadPerMeter);
        } else {
            return state;
        }
    }

    public static PathPlannerState apply(PathPlannerState state) {
        if (shouldFlip()) {
            // Create a new state so that we don't overwrite the original
            PathPlannerState transformedState = new PathPlannerState();

            Translation2d transformedTranslation = new Translation2d(FieldConstants.fieldLength - state.poseMeters.getX(), state.poseMeters.getY());
            Rotation2d transformedHeading = apply(state.poseMeters.getRotation());
            Rotation2d transformedHolonomicRotation = apply(state.holonomicRotation);

            transformedState.timeSeconds = state.timeSeconds;
            transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
            transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
            transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
            transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
            transformedState.holonomicRotation = transformedHolonomicRotation;
            transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
            transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

            return transformedState;
        } else {
            return state;
        }
    }

    public static PathPlannerTrajectory apply(PathPlannerTrajectory trajectory) {
        if (shouldFlip()) {
            List<State> transformedStates = new ArrayList<>();

            for (State s : trajectory.getStates()) {
                PathPlannerState state = (PathPlannerState) s;

                transformedStates.add(apply(state));
            }

            return new PathPlannerTrajectory(
                    transformedStates,
                    trajectory.getMarkers(),
                    trajectory.getStartStopEvent(),
                    trajectory.getEndStopEvent(),
                    trajectory.fromGUI);
        } else {
            return trajectory;
        }
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }
}
