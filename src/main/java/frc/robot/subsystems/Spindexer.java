package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

    private CANBus canBus = new CANBus("CAN");

    private StatusSignal<AngularVelocity> SpindexerVelocity;
    private StatusSignal<AngularVelocity> UptakeVelocity;

    private VelocityVoltage VelocityControl = new VelocityVoltage(0);

    private TalonFX SpindexerMotor;
    private TalonFX UptakeMotor;
    private MotorOutputConfigs outfitConfigs = new MotorOutputConfigs();
    private CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

    private boolean SpindexerManualOverride = false;
    private double SpindexerManualOverrideValue = 0.0;
    private double UptakeManualOverrideValue = 0.0;

    private double SpindexerGearRatio = 6.3; 
    private double UptakeGearRatio = 34.0/12.0; 

    private double SpindexerSpeed = 2.0;

    public Spindexer() {
        SpindexerMotor = new TalonFX(9, canBus);
        UptakeMotor = new TalonFX(15, canBus);
        SpindexerMotor.getConfigurator().apply(outfitConfigs);
        SpindexerMotor.getConfigurator().apply(limitsConfigs);
        UptakeMotor.getConfigurator().apply(outfitConfigs);
        UptakeMotor.getConfigurator().apply(limitsConfigs);
        SpindexerVelocity = SpindexerMotor.getVelocity();
        UptakeVelocity = UptakeMotor.getVelocity();
        SmartDashboard.putNumber("UptakeManualOverrideValue", UptakeManualOverrideValue);
        SmartDashboard.putNumber("SpindexerManualOverrideValue", SpindexerManualOverrideValue);
        SmartDashboard.putBoolean("SpindexerManualOverride", SpindexerManualOverride);
    }
    public void periodic() {
        BaseStatusSignal.refreshAll(SpindexerVelocity,UptakeVelocity);
    }
    
    public double SpindexerVelocity() {
    return SpindexerVelocity.getValueAsDouble() / SpindexerGearRatio;
    }

    public double UptakeVelocity() {
    return UptakeVelocity.getValueAsDouble() / UptakeGearRatio;
    }

    public Command Spin(){
        return run(()->{SpindexerMotor.setControl(VelocityControl.withVelocity(SpindexerSpeed*SpindexerGearRatio));
        UptakeMotor.setControl(VelocityControl.withVelocity(2*SpindexerSpeed*UptakeGearRatio));});
    }

    public Command Unjam(){
        return run(()->SpindexerMotor.setControl(VelocityControl.withVelocity(-0.5*SpindexerSpeed*SpindexerGearRatio)));
    }

    public Command SpindexerManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("SpindexerManualOverride", SpindexerManualOverride)){
            SpindexerMotor.setControl(VelocityControl.withVelocity(SmartDashboard.getNumber("SpindexerManualOverrideValue", SpindexerManualOverrideValue)*SpindexerGearRatio));
            UptakeMotor.setControl(VelocityControl.withVelocity(SmartDashboard.getNumber("UptakeManualOverrideValue", UptakeManualOverrideValue)*UptakeGearRatio));
         }

        else {
            SpindexerManualOverrideValue = SpindexerVelocity();
            UptakeManualOverrideValue = UptakeVelocity();
            SmartDashboard.putNumber("SpindexerManualOverrideValue", SpindexerManualOverrideValue);
            SmartDashboard.putNumber("UptakeManualOverrideValue", UptakeManualOverrideValue);
         }
      });
  }

   public Command UptakeManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("SpindexerManualOverride", SpindexerManualOverride)){

         }

        else {
            UptakeManualOverrideValue = UptakeVelocity();
            SmartDashboard.putNumber("UptakeManualOverrideValue", UptakeManualOverrideValue);
         }
      });
  }
}
