package frc.robot.subsystems;

//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private TalonFX SpinnerMotor;
    private MotorOutputConfigs outfitConfigs = new MotorOutputConfigs();
    private CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

    public Spindexer() {
        SpinnerMotor = new TalonFX(9);
        SpinnerMotor.getConfigurator().apply(outfitConfigs);
        SpinnerMotor.getConfigurator().apply(limitsConfigs);

    }
    public void periodic() {
        
    }
}
