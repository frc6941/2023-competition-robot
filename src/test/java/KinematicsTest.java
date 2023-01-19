import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class KinematicsTest {
    public static void main(String[] args) {
        Pose3d robotCenter = new Pose3d(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0.0)));
        Transform3d transformFromRobotCentertoLowTowerPivot = new Transform3d(
            new Pose3d(), new Pose3d(0.50, 0, 0, new Rotation3d())
        );
        Transform3d transformFromLowTowerPivotToTopTowerPivot = new Transform3d(
            new Pose3d(), new Pose3d(-0.60, 0.0, 0.80, new Rotation3d())
        );

        double armLength = 0.80;
        double armAngle = 0.0;
        Transform3d transformFromArmPivotToEffector = new Transform3d(
            new Translation3d(armLength, new Rotation3d(0.0, Units.degreesToRadians(armAngle), 0.0)),
            new Rotation3d(0.0, Units.degreesToRadians(armAngle), 0.0)
        );

        Pose3d endEffector = robotCenter.plus(transformFromRobotCentertoLowTowerPivot).plus(transformFromLowTowerPivotToTopTowerPivot).plus(transformFromArmPivotToEffector);
        System.out.println(endEffector);
    }
}
