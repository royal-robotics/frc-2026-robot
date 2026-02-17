package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private CANBus canBus = new CANBus("CAN");

    //motors
    private TalonFX IntakeMotorLift;
    private TalonFX IntakeMotorSpin;

    //motor configs
    private MotorOutputConfigs IntakeMotorConfig= new MotorOutputConfigs();
    private CurrentLimitsConfigs IntakeCurrentConfig= new CurrentLimitsConfigs();
    private Slot0Configs IntakePidConfigs= new Slot0Configs();
  
    //CAN configs
    private MagnetSensorConfigs IntakeCANMagnetSensor= new MagnetSensorConfigs();
    private CANcoder IntakeLiftEncoder;

    private double DeployGearRatio = (34.0/12.0)*(54.0/16.0);
    private double IntakeGearRatio = 2.0;

    //Intake classifier
    public Intake() {
        /*IntakeMotorLift = new TalonFX(0,canBus);
            IntakeMotorSpin.getConfigurator().apply(IntakeMotorConfig);
            IntakeMotorSpin.getConfigurator().apply(IntakeCurrentConfig);
        
        IntakeMotorSpin = new TalonFX(0);
            IntakeMotorLift.getConfigurator().apply(IntakeMotorConfig);
            IntakeMotorLift.getConfigurator().apply(IntakeCurrentConfig);
            IntakeMotorLift.getConfigurator().apply(IntakePidConfigs);

        IntakeLiftEncoder = new CANcoder(0);
            IntakeLiftEncoder.getConfigurator().apply(IntakeCANMagnetSensor);*/
    }

    public void periodic () {
        
    }
}
