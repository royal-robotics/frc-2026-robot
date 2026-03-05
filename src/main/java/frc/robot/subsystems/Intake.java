package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase{

    private CANBus canBus = new CANBus("CAN");

    //motors
    private TalonFX IntakeMotorLift;
    private TalonFX IntakeMotorSpin;

    //private StatusSignal<Angle> LiftSignal;
    private StatusSignal<AngularVelocity> SpinSignal;
    private StatusSignal<Angle> MotorLiftSignal;

    private PositionVoltage PositionControl = new PositionVoltage(0.0);
    private VelocityVoltage VelocityControl = new VelocityVoltage(0.0);

    //motor configs
    private MotorOutputConfigs IntakeMotorConfig= new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    private CurrentLimitsConfigs IntakeCurrentConfig= new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true);
    //private FeedbackConfigs intakeFeedbackConfigs= new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder).withFeedbackRemoteSensorID(7);
    //test values
    private Slot0Configs IntakeLiftPidConfigs= new Slot0Configs().withKS(0.08).withKV(0.1).withKA(0.001).withKP(40.0).withKD(0);
    private Slot0Configs IntakeSpinPidConfigs= new Slot0Configs().withKS(0.05).withKV(0.1).withKA(0.001).withKP(0.1).withKD(0);
  
    //CAN configs
    private MagnetSensorConfigs IntakeCANMagnetSensor= new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(Degrees.of(270)).withMagnetOffset(Degrees.of(-18.98));
    //private CANcoder IntakeLiftEncoder;

    private double DeployGearRatio = (34.0/12.0)*(54.0/16.0);
    private double IntakeGearRatio = 2.0;

    private boolean IntakeOverride = false;
    private double IntakeLiftOverride = 0.0;
    private double IntakeSpinOverride = 0.0;

    private double IntakeDown = 276.0;
    private double IntakeUp = 15.0;

    private double IntakeSpinGo = 35.0;
    private double IntakeSpinNo = 0.0;

    private boolean DeployStatus = false;

    //Intake classifier
    public Intake() {
        IntakeMotorLift = new TalonFX(16,canBus);
            IntakeMotorLift.getConfigurator().apply(IntakeMotorConfig.withNeutralMode(NeutralModeValue.Brake));
            IntakeMotorLift.getConfigurator().apply(IntakeCurrentConfig.withStatorCurrentLimit(60.0));
            IntakeMotorLift.getConfigurator().apply(IntakeLiftPidConfigs);
        
        IntakeMotorSpin = new TalonFX(17,canBus);
            IntakeMotorSpin.getConfigurator().apply(IntakeMotorConfig);
            IntakeMotorSpin.getConfigurator().apply(IntakeCurrentConfig);
            IntakeMotorSpin.getConfigurator().apply(IntakeSpinPidConfigs);

        //IntakeLiftEncoder = new CANcoder(7,canBus);
            //IntakeLiftEncoder.getConfigurator().apply(IntakeCANMagnetSensor);

        //LiftSignal = IntakeLiftEncoder.getAbsolutePosition();
        MotorLiftSignal = IntakeMotorLift.getPosition();
        SpinSignal = IntakeMotorSpin.getVelocity();

        MotorLiftSignal.waitForUpdate(0.02);
        //IntakeMotorLift.setPosition(Degrees.of(LiftSignal.getValueAsDouble()*DeployGearRatio));
        IntakeMotorLift.setPosition(0.0);
        IntakeMotorLift.setControl(PositionControl.withPosition(0.0));

        SmartDashboard.putBoolean("IntakeOverride", IntakeOverride);
        SmartDashboard.putNumber("IntakeLiftOverride", IntakeLiftOverride);
        SmartDashboard.putNumber("IntakeSpinOverride", IntakeSpinOverride);

        setDefaultCommand(IntakeManual());
    }

    public double IntakeLift() {
        return MotorLiftSignal.getValue().in(Degrees)/DeployGearRatio;
    }

    /*public double IntakeEncoderLift() {
        return LiftSignal.getValue().in(Degrees);
    }*/

    public double IntakeSpin() {
        return SpinSignal.getValueAsDouble()/IntakeGearRatio;
    }

    public Command IntakeManual() {
        return run(() -> {
        if (SmartDashboard.getBoolean("IntakeOverride", IntakeOverride)){
            IntakeMotorLift.setControl(PositionControl.withPosition(Degrees.of(SmartDashboard.getNumber("IntakeLiftOverride", IntakeLiftOverride))));
            IntakeMotorSpin.setControl(VelocityControl.withVelocity(SmartDashboard.getNumber("IntakeSpinOverride", IntakeSpinOverride)*IntakeGearRatio));
         }

        else {
            IntakeLiftOverride = IntakeLift();
            IntakeSpinOverride = IntakeSpin();
            SmartDashboard.putNumber("IntakeLiftOverride", IntakeLiftOverride);
            SmartDashboard.putNumber("IntakeSpinOverride", IntakeSpinOverride);
         }
        });
    }

    public Command IntakeDeploy(){
        return runOnce(()->{ 
            if (DeployStatus == false) {
            IntakeMotorLift.setControl(PositionControl.withPosition(Degrees.of(IntakeDown*DeployGearRatio)));DeployStatus = true;
        }
            else {
            IntakeMotorLift.setControl(PositionControl.withPosition(Degrees.of(IntakeUp*DeployGearRatio)));DeployStatus = false;
        }
    });
    }

    public Command SpinIntake(){
        return runEnd(()->IntakeMotorSpin.setControl(VelocityControl.withVelocity(IntakeSpinGo)),()->IntakeMotorSpin.setControl(VelocityControl.withVelocity(IntakeSpinNo)));
    }


    //startEnd(()->{IntakeMotorLift.setControl(PositionControl.withPosition(Degrees.of(IntakeDown)));}, ()->{IntakeMotorLift.setControl(PositionControl.withPosition(Degrees.of(IntakeUp)));});

    //IntakeMotorSpin.setControl(VelocityControl.withVelocity(IntakeSpinGo*IntakeGearRatio));
    //IntakeMotorSpin.setControl(VelocityControl.withVelocity(IntakeSpinNo));

    public void periodic () {
        BaseStatusSignal.refreshAll(/*LiftSignal,*/ SpinSignal, MotorLiftSignal);
    }
}
