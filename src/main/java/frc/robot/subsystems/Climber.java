package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import pabeles.concurrency.ConcurrencyOps.NewInstance;

public class Climber extends SubsystemBase{
   
    private TalonFX ClimberMotor; 
    
    //motor configs
    private MotorOutputConfigs IntakeMotorConfig= new MotorOutputConfigs();
    private CurrentLimitsConfigs IntakeCurrentConfig= new CurrentLimitsConfigs();

    private PositionVoltage ClimberPosition = new PositionVoltage(Degrees.of(0)); //ratio is 1:25

    public Climber() {
      ClimberMotor = new TalonFX(0);
        ClimberMotor.getConfigurator().apply (IntakeMotorConfig);
        ClimberMotor.getConfigurator().apply(IntakeCurrentConfig);
  }

  public Command ClimberUPPP(){
      return runOnce(()->ClimberMotor.setControl(ClimberPosition.withPosition(Degrees.of(180))));
  }

  public Command ClimberDooooown(){
      return runOnce(()->ClimberMotor.setControl(ClimberPosition.withPosition(Degrees.of(0))));
  }

  public Command ClimberManual(){
      return run(()->{
        if (SmartDashboard.getBollean("ClimberManualOverride", ClimberManualOverride)){
            ClimberMotor.setControl(ClimberPosition.withPosition(SmartDashboard.getNumber("ClimberManualOverrideValue", ClimberManualOverride)));
         }

        else {
            ClimberManualOverrideValue = ClimberPosition();
            SmartDashboard.putNumber("ClimberManualOverrideValue", ClimberManualOverrideValue);
         }
      })
  }

  public void periodic() {
  }
}

