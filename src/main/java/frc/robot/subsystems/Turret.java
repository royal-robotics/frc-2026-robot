package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class Turret extends SubsystemBase{

    private CANBus canBus = new CANBus("CAN");

    private TalonFX TurretAngleMotor;
    private TalonFX TurretHoodMotor;
    private TalonFX TurretShooterMotor;
    private TalonFX TurretShooterFollowerMotor;

    private CANcoder TurretAngleSmall;
    private CANcoder TurretAngleBig;

    private MotorOutputConfigs outfitConfigs = new MotorOutputConfigs();
    private CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true);
    private Slot0Configs TurretPIDConfigs = new Slot0Configs().withKS(0.16433).withKV(0.11742).withKA(0.0061442).withKP(10.0).withKD(0);
    private Slot0Configs ShooterPIDConfigs = new Slot0Configs().withKS(0.16433).withKV(0.11742).withKA(0.0061442).withKP(0.3).withKD(0); // inital p 0.17107
    private Slot0Configs HoodPIDConfigs= new Slot0Configs().withKS(0.08).withKV(0.1).withKA(0.001).withKP(40.0).withKD(0);
    private MagnetSensorConfigs magnetConfigsSmall = new MagnetSensorConfigs().withMagnetOffset(Degrees.of(-59.94)).withAbsoluteSensorDiscontinuityPoint(Degrees.of(360.0));
    private MagnetSensorConfigs magnetConfigsBig = new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive).withMagnetOffset(Degrees.of(-6.67)).withAbsoluteSensorDiscontinuityPoint(Degrees.of(360.0));

    private StatusSignal<Angle> turretAngleSignal;
    private StatusSignal<Angle> turretHoodSignal;
    private StatusSignal<AngularVelocity> turretShooterSignal;
    private StatusSignal<Angle> turretAngleSmallSignal;
    private StatusSignal <Angle> turretAngelBigSignal;

    private PositionVoltage positionControl = new PositionVoltage(Rotations.of(0.0));
    private VelocityVoltage velocityControl = new VelocityVoltage(RotationsPerSecond.of(0.0));
    private Follower followControl = new Follower(11, MotorAlignmentValue.Aligned);
    private VoltageOut voltageOut = new VoltageOut(0.0);

    private boolean TurretOverride = false;
    private double TurretAngleOverride = 0.0;
    private double TurretHoodOverride = 0.0;
    private double TurretShooterOverride = 0.0;

    private double TurretGearRatio = (56.0/12.0)*(10.0);
    private double HoodGearRatio = (40.0/15.0)*(30.0/18.0)*(197.0/10.0);
    private double ShooterGearRatio = (32.0/16.0)*(18.0/36.0)*(16.0/16.0);
    private double BigTurretEncoderRatio = 23.0/100.0;
    private double SmallTurretEncoderRatio = 19.0/100.0;

    private double TurretAngleRatio = TurretGearRatio/360.0;
    private double HoodAngleRatio = HoodGearRatio/360.0;

    private double ShooterSpeed = 80.0;
    private double IdealHoodAngle = 20.0;
    private double MotorHoodAngle = IdealHoodAngle-12.75;

    private double RealTurretAngle = 0.0;
    private double TurretAngleError = 0.0;

    private double TurretMin = 5.0;
    private double TurretMax = 400.0;

    private double HoodMin = 0.0;
    
    private double HoodMax = 26.0;

    private double CalculatedDistance;

    private final SysIdRoutine ShooterPID = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            null, // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("ShooterPID_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> TurretShooterMotor.setControl(voltageOut.withOutput(output)),
            null,
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return ShooterPID.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return ShooterPID.dynamic(direction);
    }

    
    public Turret() {
        TurretAngleMotor = new TalonFX(13, canBus);
        TurretHoodMotor = new TalonFX(10, canBus);
        TurretShooterMotor = new TalonFX(11, canBus);
        TurretShooterFollowerMotor = new TalonFX(12,canBus);

        TurretAngleMotor.getConfigurator().apply(outfitConfigs.withNeutralMode(NeutralModeValue.Brake));
        TurretHoodMotor.getConfigurator().apply(outfitConfigs.withNeutralMode(NeutralModeValue.Brake));
        TurretShooterMotor.getConfigurator().apply(outfitConfigs.withNeutralMode(NeutralModeValue.Coast));
        TurretShooterFollowerMotor.getConfigurator().apply(outfitConfigs.withNeutralMode(NeutralModeValue.Coast));

        TurretAngleMotor.getConfigurator().apply(limitsConfigs);
        TurretHoodMotor.getConfigurator().apply(limitsConfigs);
        TurretShooterMotor.getConfigurator().apply(limitsConfigs);
        TurretShooterFollowerMotor.getConfigurator().apply(limitsConfigs);

        TurretShooterMotor.getConfigurator().apply(ShooterPIDConfigs);
        TurretHoodMotor.getConfigurator().apply(HoodPIDConfigs);
        TurretAngleMotor.getConfigurator().apply(TurretPIDConfigs);

        TurretAngleSmall = new CANcoder(5,canBus);
        TurretAngleBig = new CANcoder(6,canBus);
        TurretAngleSmall.getConfigurator().apply(magnetConfigsSmall);
        TurretAngleBig.getConfigurator().apply(magnetConfigsBig);

        turretAngleSignal = TurretAngleMotor.getPosition();
        turretHoodSignal = TurretHoodMotor.getPosition();
        turretShooterSignal = TurretShooterMotor.getVelocity();

        turretShooterSignal.waitForUpdate(0.02);
        TurretShooterFollowerMotor.setControl(followControl);

        turretHoodSignal.waitForUpdate(0.02);
        TurretHoodMotor.setPosition(0.0);

        turretAngleSmallSignal = TurretAngleSmall.getAbsolutePosition();
        turretAngelBigSignal = TurretAngleBig.getAbsolutePosition();

        GetTurretAngle();
        TurretAngleMotor.setPosition(Degrees.of(RealTurretAngle*TurretGearRatio));

        SmartDashboard.putBoolean("TurretManualOverride", TurretOverride);
        SmartDashboard.putNumber("TurretAngleOverride", TurretAngleOverride);
        SmartDashboard.putNumber("TurretHoodOverride", TurretHoodOverride);
        SmartDashboard.putNumber("TurretShooterOverride", TurretShooterOverride);

        setDefaultCommand(TurretManual());
    }

    public void getRobotDistance(double RobotDistace) {
        CalculatedDistance = RobotDistace;
    }

    public double TurretAngle() {
        return turretAngleSignal.getValue().in(Degrees)/TurretGearRatio;
    }

    public double TurretHood() {
        return turretHoodSignal.getValue().in(Degrees)/HoodGearRatio;
    }

    public double TurretShooter() {
        return turretShooterSignal.getValueAsDouble()/ShooterGearRatio;
    }

     public double TurretEncoderBig() {
        return turretAngelBigSignal.getValue().in(Degrees);
    }

     public double TurretEncoderSmall() {
        return turretAngleSmallSignal.getValue().in(Degrees);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(turretAngleSignal, turretHoodSignal, turretShooterSignal,turretAngelBigSignal,turretAngleSmallSignal);
        //GetTurretAngle();
    }

public Command Shoot(){
    return startEnd(()->TurretShooterMotor.setControl(velocityControl.withVelocity(ShooterSpeed*ShooterGearRatio)),()->TurretShooterMotor.setControl(velocityControl.withVelocity(0.0*ShooterGearRatio)));
}

//public Command LockPosition(){
   // return runOnce(()->TurretAngleMotor.setPosition(MotorTurretAngle));
//}

public Command HoodStepUp(){
    return runOnce(()->TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of((TurretHood()+0.25)*HoodGearRatio)))).onlyIf(()->TurretHood()<HoodMax);
}

public Command HoodStepDown(){
    return runOnce(()->TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of((TurretHood()-0.25)*HoodGearRatio)))).onlyIf(()->TurretHood()>HoodMin);
}

public Command TurretRotateRight(){
    return runOnce(()->TurretAngleMotor.setControl(positionControl.withPosition(Degrees.of((TurretAngle()-1)*TurretGearRatio)))).onlyIf(()->TurretAngle()>TurretMin);
}

public Command TurretRotateLeft(){
    return runOnce(()->TurretAngleMotor.setControl(positionControl.withPosition(Degrees.of((TurretAngle()+1)*TurretGearRatio)))).onlyIf(()->TurretAngle()<TurretMax);
}

  public Command TurretManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("TurretManualOverride", TurretOverride)){
            TurretAngleMotor.setControl(positionControl.withPosition(Degrees.of(SmartDashboard.getNumber("TurretAngleOverride", TurretAngleOverride)*TurretGearRatio)));
            TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of(SmartDashboard.getNumber("TurretHoodOverride", TurretHoodOverride)*HoodGearRatio)));
            TurretShooterMotor.setControl(velocityControl.withVelocity(SmartDashboard.getNumber("TurretShooterOverride", TurretShooterOverride)*ShooterGearRatio));
         }

        else {
            TurretAngleOverride = TurretAngle();
            TurretHoodOverride = (TurretHood());//.0855294*CalculatedDistance)-3.42198;
            TurretShooterOverride = (TurretShooter());//0.000880253*CalculatedDistance*CalculatedDistance)-(0.0716505*CalculatedDistance)+31.81023;
            SmartDashboard.putNumber("TurretAngleOverride", TurretAngleOverride);
            SmartDashboard.putNumber("TurretHoodOverride", TurretHoodOverride);
            SmartDashboard.putNumber("TurretShooterOverride", TurretShooterOverride);

            //TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of(TurretHoodOverride*HoodGearRatio)));
            //TurretShooterMotor.setControl(velocityControl.withVelocity(TurretShooterOverride*ShooterGearRatio));
        }});
  }

  private void GetTurretAngle(){
    BaseStatusSignal.waitForAll(0.02,turretAngelBigSignal,turretAngleSmallSignal);
    double BigAngle = TurretEncoderBig();
    double SmallAngle = TurretEncoderSmall();
    double BigEncoderRotations = BigAngle/360.0;
    double SmallEncoderRotations = SmallAngle/360.0;
    ArrayList <Double> SmallEncoder = new ArrayList<Double>();
    ArrayList <Double> BigEncoder = new ArrayList<Double>() ;
    for (int i = 0 ; i <= 8 ; i++){
        SmallEncoder.add((SmallEncoderRotations+i)*SmallTurretEncoderRatio*360.0);
        BigEncoder.add((BigEncoderRotations+i)*BigTurretEncoderRatio*360.0);
    }
    int BigPointer = 0;
    int SmallPointer = 0;
    boolean finished = false;
    double Small = SmallEncoder.get(SmallPointer);
    double Big = BigEncoder.get(BigPointer);
    double FinalAngle = ((Small+Big)/2.0);
    double FinalError = Math.abs(Big-Small);
    while (!finished){
    Small = SmallEncoder.get(SmallPointer);
    Big = BigEncoder.get(BigPointer);
    double PossibleAngle = ((Small+Big)/2.0);
    double PossibleError = Math.abs(Big-Small);
    if (PossibleError < FinalError){
    FinalAngle = PossibleAngle;
    FinalError = PossibleError;
    }
    if (BigPointer >= BigEncoder.size()-1 || SmallPointer >= SmallEncoder.size()-1){
        finished = true;
    } else {
        if (Big < Small){
            BigPointer++;
        } else {
            SmallPointer++;
        }
    }
    RealTurretAngle = FinalAngle;
    TurretAngleError = FinalError;


  }
}}
