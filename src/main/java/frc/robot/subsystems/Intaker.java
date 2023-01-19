package frc.robot.subsystems;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyVictorSPX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Intaker implements Updatable{
    public static class PeriodicIO {
        // INPUT
        public double intakerMotorVoltage = 0.0;
        public boolean hasGamePiece = false;
    
        // OUTPUT
        public double intakerMotorDemand = 1.0;
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazyVictorSPX intakerMotor = new LazyVictorSPX(Constants.CANID.INTAKER_MOTOR);
    private final AnalogInput gamepieceSensor = new AnalogInput(Constants.ANALOG_ID.GAMEPIECE_SENSOR);

    private static Intaker instance;

    public static Intaker getInstance() {
        if (instance == null) {
            instance = new Intaker();
        }
        return instance;
    }
    
    private Intaker() {
        intakerMotor.setNeutralMode(NeutralMode.Brake);
        intakerMotor.setInverted(false);
    }

    public void setIntakerPower(double power) {
        mPeriodicIO.intakerMotorDemand = Util.clamp(power, -1.0, 1.0);
    }

    public double getIntakerVoltage() {
        return mPeriodicIO.intakerMotorVoltage;
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.intakerMotorVoltage = intakerMotor.getMotorOutputVoltage();
        mPeriodicIO.hasGamePiece = gamepieceSensor.getAverageVoltage() < 2.0 ? true : false;
    }
    
    @Override
    public synchronized void update(double time, double dt){
    }
    
    @Override
    public synchronized void write(double time, double dt){
        if(!mPeriodicIO.hasGamePiece || (mPeriodicIO.hasGamePiece && mPeriodicIO.intakerMotorDemand <= 0.0)){
            intakerMotor.set(ControlMode.PercentOutput, mPeriodicIO.intakerMotorDemand);
        } else {
            intakerMotor.set(ControlMode.PercentOutput, 0.0);
        }
        System.out.println(intakerMotor.getMotorOutputPercent());
    }
    
    @Override
    public synchronized void telemetry(){

    }
    
    @Override
    public synchronized void start(){
    }
    
    @Override
    public synchronized void stop(){
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        
    }

    @Override
    public synchronized void simulate(){
    }
}
