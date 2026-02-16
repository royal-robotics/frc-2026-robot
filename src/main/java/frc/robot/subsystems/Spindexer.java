package frc.robot.subsystems;

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

    public Spindexer() {
        SpindexerMotor = new TalonFX(9);
        UptakeMotor = new TalonFX(15);
        SpindexerMotor.getConfigurator().apply(outfitConfigs);
        SpindexerMotor.getConfigurator().apply(limitsConfigs);
        UptakeMotor.getConfigurator().apply(outfitConfigs);
        UptakeMotor.getConfigurator().apply(limitsConfigs);
        SpindexerVelocity = SpindexerMotor.getVelocity();
        UptakeVelocity = UptakeMotor.getVelocity();
    }
    public void periodic() {
    }
    
    public double SpindexerVelocity() {
    return SpindexerVelocity.getValueAsDouble();
    }

    public double UptakeVelocity() {
    return UptakeVelocity.getValueAsDouble();
    }

    public Command SpindexerManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("SpindexerManualOverride", SpindexerManualOverride)){
            SpindexerMotor.setControl(VelocityControl.withVelocity(SmartDashboard.getNumber("SpindexerManualOverrideValue", SpindexerManualOverrideValue)));
         }

        else {
            SpindexerManualOverrideValue = SpindexerVelocity();
            SmartDashboard.putNumber("SpindexerManualOverrideValue", SpindexerManualOverrideValue);
         }
      });
  }

   public Command UptakeManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("SpindexerManualOverride", SpindexerManualOverride)){
            UptakeMotor.setControl(VelocityControl.withVelocity(SmartDashboard.getNumber("UptakeManualOverrideValue", UptakeManualOverrideValue)));
         }

        else {
            UptakeManualOverrideValue = UptakeVelocity();
            SmartDashboard.putNumber("UptakeManualOverrideValue", UptakeManualOverrideValue);
         }
      });
  }
}
