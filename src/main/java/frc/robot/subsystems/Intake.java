package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
   
    private TalonFX IntakeMotorLift;
    private TalonFX IntakeMotorSpin; //motors

    private CANcoder IntakeLiftEncoder;
    
    private StatusSignal IntakeLiftData;

    public Intake() {
        IntakeMotorLift = new TalonFX(0);
        IntakeMotorSpin = new TalonFX(0);

        IntakeLiftEncoder = new CANcoder(0);

        IntakeLiftData = IntakeMotorLift.getPosition ();
    
    }

    public void periodic () {
        
    }
}
