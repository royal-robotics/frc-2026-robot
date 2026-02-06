package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

public class Climber extends SubsystemBase{
   
    private TalonFX ClimberMotor; // wench motor

    public Climber() {
        ClimberMotor = new TalonFX(0);
  }

  public void periodic() {
  }
}

