package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

public class Climber extends SubsystemBase{
   
    private TalonFX ClimberMotor; 
    
    //motor configs
    private MotorOutputConfigs IntakeMotorConfig= new MotorOutputConfigs();
    private CurrentLimitsConfigs IntakeCurrentConfig= new CurrentLimitsConfigs();

    public Climber() {
      ClimberMotor = new TalonFX(0);
        ClimberMotor.getConfigurator().apply (IntakeMotorConfig);
        ClimberMotor.getConfigurator().apply(IntakeCurrentConfig);
  }

  public void periodic() {
  }
}

