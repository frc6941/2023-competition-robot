package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Intaker implements Updatable{
    @AutoLog
    public static class IntakerPeriodicIO {
        // INPUT
        public double intakerMotorVoltage = 0.0;
        public boolean hasGamePiece = false;
    
        // OUTPUT
        public double intakerMotorDemand = 0.0;
    }
    
    public IntakerPeriodicIOAutoLogged mPeriodicIO = new IntakerPeriodicIOAutoLogged();

    private final CANSparkMax intakerMotor = new CANSparkMax(Constants.CANID.INTAKER_MOTOR, MotorType.kBrushless);
    private final AnalogInput gamepieceSensor = new AnalogInput(Constants.ANALOG_ID.GAMEPIECE_SENSOR);

    private static Intaker instance;

    public static Intaker getInstance() {
        if (instance == null) {
            instance = new Intaker();
        }
        return instance;
    }
    
    private Intaker() {
        intakerMotor.restoreFactoryDefaults();
        intakerMotor.setIdleMode(IdleMode.kBrake);
        intakerMotor.setSmartCurrentLimit(25, 15);
    }

    public void setIntakerPower(double power) {
        mPeriodicIO.intakerMotorDemand = Util.clamp(power, -1.0, 1.0);
    }

    public double getIntakerVoltage() {
        return mPeriodicIO.intakerMotorVoltage;
    }

    public boolean hasGamePiece() {
        return mPeriodicIO.hasGamePiece;
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.intakerMotorVoltage = intakerMotor.getAppliedOutput();
        mPeriodicIO.hasGamePiece = false;
    }
    
    @Override
    public synchronized void update(double time, double dt){
    }
    
    @Override
    public synchronized void write(double time, double dt){
        if(!mPeriodicIO.hasGamePiece || (mPeriodicIO.hasGamePiece && mPeriodicIO.intakerMotorDemand <= 0.0)){
            intakerMotor.set(mPeriodicIO.intakerMotorDemand);
        } else {
            intakerMotor.set(0.0);
        }
    }
    
    @Override
    public synchronized void telemetry(){
        Logger.getInstance().processInputs("Intaker", mPeriodicIO);
    }
    
    @Override
    public synchronized void start(){
    }
    
    @Override
    public synchronized void stop(){
    }

    @Override
    public synchronized void simulate(double time, double dt){
    }
}
