// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.server.PathPlannerServer;

import org.frcteam6941.looper.UpdateManager;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.coordinators.Coordinator;
import frc.robot.shuffleboard.ShuffleBoardInteractions;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private UpdateManager updateManager;
    private final AutoSelector mAutoSelector = AutoSelector.getInstance();
    private final ShuffleBoardInteractions mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject");

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.getInstance().addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
        }

        Logger.getInstance().start();

        this.updateManager = new UpdateManager(
                Intaker.getInstance(),
                ArmAndExtender.getInstance(),
                SJTUSwerveMK5Drivebase.getInstance(),
                Coordinator.getInstance());
        this.updateManager.startEnableLoop(Constants.LOOPER_DT);

        CameraServer.startAutomaticCapture();
        if (Constants.AUTO_TUNING) {
            PathPlannerServer.startServer(6941);
        }
    }

    @Override
    public void robotPeriodic() {
        mShuffleBoardInteractions.update();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        this.updateManager.stopEnableLoop();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        this.updateManager.startDisableLoop(Constants.LOOPER_DT);
    }

    @Override
    public void disabledPeriodic() {
        mAutoSelector.updateModeCreator();
    }

    /**
     * This autonomous runs the autonomous command selected.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        this.updateManager.stopDisableLoop();

        CommandScheduler.getInstance().enable();
        this.updateManager.startEnableLoop(Constants.LOOPER_DT);

        Optional<AutoModeBase> autoMode = mAutoSelector.getAutoMode();
        autoMode.ifPresent(autoModeBase -> {
            SJTUSwerveMK5Drivebase.getInstance().getLocalizer().reset(autoModeBase.getStartingPose());
            if (autoModeBase.getAutoCommand() != null) {
                autoModeBase.getAutoCommand().schedule();
            }
        });
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        this.updateManager.stopDisableLoop();
        CommandScheduler.getInstance().enable();
        this.updateManager.startEnableLoop(Constants.LOOPER_DT);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        Coordinator.getInstance().updateDriverAndOperatorCommand();
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {
        updateManager.runAllSimulate();
    }
}
