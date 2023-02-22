package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveTeleopCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSupersturcture;
    Supplier<Translation2d> translationSupplier;
    Supplier<Double> rotationSupplier;
    boolean isOpenLoop;

    private Translation2d lastTranslation = new Translation2d();
    private double lastRotation = 0.0;

    private double linearAccMax = 25.0;
    private double linearAccMin = 3.0;

    public DriveTeleopCommand(
            SJTUSwerveMK5Drivebase mDrivebase,
            ArmAndExtender mSuperstructure,
            Supplier<Translation2d> translationSupplier,
            Supplier<Double> rotationSupplier, boolean isOpenLoop) {
        this.mDrivebase = mDrivebase;
        this.mSupersturcture = mSuperstructure;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.isOpenLoop = isOpenLoop;
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Translation2d translation = translationSupplier.get();
        double rotation = rotationSupplier.get();

        if(!isOpenLoop) {
            
        }

        mDrivebase.setLockHeading(false);
        mDrivebase.drive(translation, rotation, true, isOpenLoop);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
