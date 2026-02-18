package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Spindexer extends SubsystemBase {

    private CANBus canBus = new CANBus("CAN");

    private StatusSignal<AngularVelocity> SpindexerVelocity;
    private StatusSignal<AngularVelocity> UptakeVelocity;

    private VelocityVoltage VelocityControl = new VelocityVoltage(0);
    private VoltageOut voltageOut = new VoltageOut(0.0);

    private TalonFX SpindexerMotor;
    private TalonFX UptakeMotor;
    private MotorOutputConfigs outfitConfigs = new MotorOutputConfigs();
    private CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
    private Slot0Configs SpindexerPIDConfigs = new Slot0Configs().withKS(0.11228).withKV(0.093345).withKA(0.0016982).withKP(0.033478).withKD(0);

    private boolean SpindexerManualOverride = false;
    private double SpindexerManualOverrideValue = 0.0;
    private double UptakeManualOverrideValue = 0.0;

    private double SpindexerGearRatio = 6.3; 
    private double UptakeGearRatio = 34.0/12.0; 

    private double SpindexerSpeed = 2.0;

    private final SysIdRoutine SpindexerPID = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            null, // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SpindexerPID_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> SpindexerMotor.setControl(voltageOut.withOutput(output)),
            null,
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return SpindexerPID.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return SpindexerPID.dynamic(direction);
    }

    public Spindexer() {
        SpindexerMotor = new TalonFX(9, canBus);
        UptakeMotor = new TalonFX(15, canBus);
        SpindexerMotor.getConfigurator().apply(outfitConfigs);
        SpindexerMotor.getConfigurator().apply(limitsConfigs);
        SpindexerMotor.getConfigurator().apply(SpindexerPIDConfigs);
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
    return SpindexerVelocity.getValueAsDouble();
    }

    public double UptakeVelocity() {
    return UptakeVelocity.getValueAsDouble();
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
            UptakeMotor.setControl(VelocityControl.withVelocity(SmartDashboard.getNumber("UptakeManualOverrideValue", UptakeManualOverrideValue)*UptakeGearRatio));
         }

        else {
            UptakeManualOverrideValue = UptakeVelocity();
            SmartDashboard.putNumber("UptakeManualOverrideValue", UptakeManualOverrideValue);
         }
      });
  }
}
