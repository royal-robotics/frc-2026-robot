package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private TalonFX SpinnerMotor;
    private StatusSignal SpinSignal;

    public Spindexer() {
        SpinnerMotor = new TalonFX(0);
        SpinSignal = SpinnerMotor.getPosition();
    }
    public void periodic() {
        
    }
}
