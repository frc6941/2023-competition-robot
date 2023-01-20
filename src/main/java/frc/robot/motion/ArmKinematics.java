package frc.robot.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.states.SuperstructureState;

public class ArmKinematics {
    public static final Transform3d transformFromRobotCentertoLowTowerPivot = new Transform3d(
            new Pose3d(), new Pose3d(0.50, 0, 0, new Rotation3d()));
    public static final Transform3d transformFromLowTowerPivotToTopTowerPivot = new Transform3d(
            new Pose3d(), new Pose3d(-0.50, 0.0, 0.80, new Rotation3d()));

    public static final Pose3d forwardKinematics(SuperstructureState q, Pose2d drivetrainPose) {
        Transform3d transformFromArmPivotToEffector = new Transform3d(
                new Translation3d(q.extenderLength, new Rotation3d(0.0, q.armAngle.getRadians(), 0.0)),
                new Rotation3d(0.0, Units.degreesToRadians(q.armAngle.getRadians()), 0.0));
        Pose3d drivetrainPose3d = new Pose3d(drivetrainPose);
        return drivetrainPose3d
                .plus(transformFromRobotCentertoLowTowerPivot)
                .plus(transformFromLowTowerPivotToTopTowerPivot)
                .plus(transformFromArmPivotToEffector);
    }

    public static final SuperstructureState inverseKinematics(Translation3d endEffectorPosition, Pose2d drivetrainPose) {
        Pose3d drivetrainPose3d = new Pose3d(drivetrainPose);
        Pose3d topTowerPivot = drivetrainPose3d
                .plus(transformFromRobotCentertoLowTowerPivot)
                .plus(transformFromLowTowerPivotToTopTowerPivot);
        Transform3d endEffectorToTower = topTowerPivot.minus(new Pose3d(endEffectorPosition, new Rotation3d()));
        Rotation2d armAngle = new Rotation2d(endEffectorToTower.getX(), endEffectorToTower.getZ());
        double extenderLength = endEffectorToTower.getTranslation().getNorm();
        return new SuperstructureState(armAngle, extenderLength);
    }
}
