package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Climber extends SubsystemBase{

    private CANBus canBus = new CANBus("CAN");
   
    private TalonFX ClimberMotor; 
    
    //motor configs
    private MotorOutputConfigs IntakeMotorConfig= new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    private CurrentLimitsConfigs IntakeCurrentConfig= new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true);
    private Slot0Configs climberSlot0Configs = new Slot0Configs().withKS(0.1).withKV(0.1).withKA(0.001).withKP(5.0).withKD(0);

    private StatusSignal<Angle> ClimberPositionSignal;

    private PositionVoltage ClimberPosition = new PositionVoltage(Rotations.of(0)); //ratio is 1:25
    private boolean ClimberManualOverride = false;
    private double ClimberManualOverrideValue = 0.0;

    private double ClimberGearRatio = 25.0;
    private double ClimberDistanceRatio = ClimberGearRatio/2.36;
    private double ClimberTop = 8.0;
    private double ClimberMiddle = 2.0;
    private double ClimberBottom = 0.0;
    private double ClimberReset = -2.0;


    public Climber() {
      ClimberMotor = new TalonFX(14, canBus);
        ClimberMotor.getConfigurator().apply (IntakeMotorConfig);
        ClimberMotor.getConfigurator().apply(IntakeCurrentConfig);
        ClimberPositionSignal = ClimberMotor.getPosition();
        ClimberMotor.getConfigurator().apply(climberSlot0Configs);
        SmartDashboard.putNumber("ClimberManualOverrideValue", ClimberManualOverrideValue);
        SmartDashboard.putBoolean("ClimberManualOverride", ClimberManualOverride);
        setDefaultCommand(ClimberManual());

        ClimberPositionSignal.waitForUpdate(0.02);
        ClimberMotor.setPosition(0.0);
        ClimberMotor.setControl(ClimberPosition.withPosition(0.0));
  }

  public double ClimberPosition() {
    return ClimberPositionSignal.getValueAsDouble()/ClimberDistanceRatio;
  }

  public Command ClimberUp(){
      return runOnce(()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberTop*ClimberDistanceRatio))));
  }

  public Command ClimberDown(){
      return runOnce(()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberBottom*ClimberDistanceRatio))));
  }

  public Command ClimberToggle(){
    return startEnd(()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberTop*ClimberDistanceRatio))),()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberBottom*ClimberDistanceRatio))));
  }

  public Command AutoClimberToggle(){
    return startEnd(()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberTop*ClimberDistanceRatio))),()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberMiddle*ClimberDistanceRatio))));
  }

  public Command ClimberZero(){
    return runOnce(()->ClimberMotor.setControl(ClimberPosition.withPosition(Rotations.of(ClimberReset*ClimberDistanceRatio))));
  }

  public Command ClimberManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("ClimberManualOverride", ClimberManualOverride)){
            ClimberMotor.setControl(ClimberPosition.withPosition(SmartDashboard.getNumber("ClimberManualOverrideValue", ClimberManualOverrideValue)*ClimberDistanceRatio));
         }

        else {
            ClimberManualOverrideValue = ClimberPosition();
            SmartDashboard.putNumber("ClimberManualOverrideValue", ClimberManualOverrideValue);
         }
      });
  }

  public void periodic() {
    BaseStatusSignal.refreshAll(ClimberPositionSignal);
  }
}

