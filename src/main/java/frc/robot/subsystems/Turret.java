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
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    private Slot0Configs TurretPIDConfigs = new Slot0Configs().withKS(0.16433).withKV(0.11742).withKA(0.0061442).withKP(20.0).withKD(2.0);
    private Slot0Configs ShooterPIDConfigs = new Slot0Configs().withKS(0.16433).withKV(0.11742).withKA(0.0061442).withKP(0.6).withKD(0.0); // inital p 0.17107
    private Slot0Configs HoodPIDConfigs= new Slot0Configs().withKS(0.08).withKV(0.1).withKA(0.001).withKP(40.0).withKD(0.0);
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

    private double TurretMin = 40.0;
    private double TurretMax = 420.0;

    private double HoodMin = 0.0;
    
    private double HoodMax = 26.0;

    private double ShooterMax = 80.0;
    private double ShooterMin = 25.0;

    private SwerveDriveState TotalRobotPose;

    private double CalculatedDistance;
    private double CalculatedHood;
    private double CalculatedShooter;

    private double MovingCalculatedHood;
    private double MovingCalculatedShooter;
    private double MovingCalculatedAngle;

    private final Translation2d blueGoal = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    private final Translation2d redGoal = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));
    private final Translation2d redLeft = new Translation2d((14.0),(2.0));
    private final Translation2d redRight = new Translation2d((14.0),(6.0));
    private final Translation2d blueLeft = new Translation2d((2.5),(6.0));
    private final Translation2d blueRight = new Translation2d((2.5),(2.0));

    private final Translation2d ShooterOffset = new Translation2d(Units.inchesToMeters(4.25),Units.inchesToMeters(-3.5));

    private double RobotAngle;
    private double VectorAngle;
    private double CalculatedAngle;

    private Targets CurrentGoal = Targets.blueGoal;

    private boolean HoodDown = false;

    private boolean TargetOveride = false;


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

        setDefaultCommand(AutoTarget());
    }

    public void getRobotPose(SwerveDriveState TheRobotPose) {
        TotalRobotPose = TheRobotPose;
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

public void SwapTarget(boolean TargetSwap){
    TargetOveride = TargetSwap;
}

  public Command TurretManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("TurretManualOverride", TurretOverride)){
            TurretAngleMotor.setControl(positionControl.withPosition(Degrees.of(SmartDashboard.getNumber("TurretAngleOverride", TurretAngleOverride)*TurretGearRatio)));
            TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of(SmartDashboard.getNumber("TurretHoodOverride", TurretHoodOverride)*HoodGearRatio)));
            TurretShooterMotor.setControl(velocityControl.withVelocity(SmartDashboard.getNumber("TurretShooterOverride", TurretShooterOverride)*ShooterGearRatio));
         }

        else {
            
            SmartDashboard.putNumber("TurretAngleOverride", TurretAngleOverride);
            SmartDashboard.putNumber("TurretHoodOverride", TurretHoodOverride);
            SmartDashboard.putNumber("TurretShooterOverride", TurretShooterOverride);

            //TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of(TurretHoodOverride*HoodGearRatio)));
            //TurretShooterMotor.setControl(velocityControl.withVelocity(TurretShooterOverride*ShooterGearRatio));
        }});
  }

  public Command ChooseTarget(Targets Target){
    return runOnce(()->{
        CurrentGoal = Target;
    });
  }

  public boolean OnTarget(){
    return Math.abs(TurretAngle()-CalculatedAngle) < 30 && Math.abs(TurretHood()-CalculatedHood) < 2.5;
  }

  public Command TrenchToggle(boolean toggle) {
    return runOnce(()-> HoodDown = toggle);
  }

  public Command AutoTarget(){
    return  runEnd(()->{
            Pose2d originalPose = TotalRobotPose.Pose;
            Pose2d movingPose = originalPose.exp(TotalRobotPose.Speeds.toTwist2d(0.1));
            Translation2d ShooterVector = ShooterOffset.rotateBy(movingPose.getRotation());
            Translation2d ShooterPosition = movingPose.getTranslation().plus(ShooterVector);
            Translation2d CurrentGoalPos = new Translation2d();
            switch (CurrentGoal) {
            case redGoal:
                
                CurrentGoalPos = redGoal;
                break;
            case redAlliance:
                if (TargetOveride == false){
                if (ShooterPosition.getY() >= 4.035) {
                    CurrentGoalPos = redRight;
                } else {
                    CurrentGoalPos = redLeft;
                }
                } else {
                    if (ShooterPosition.getY() >= 4.035) {
                    CurrentGoalPos = redLeft;
                } else {
                    CurrentGoalPos = redRight;
                }
                }
                break;

            case blueGoal:
                
                CurrentGoalPos = blueGoal;
                break;
            case blueAlliance:
                
                if (TargetOveride == false){
                if (ShooterPosition.getY() >= 4.035) {
                    CurrentGoalPos = blueLeft;
                } else {
                    CurrentGoalPos = blueRight;
                }
                } else {
                    if (ShooterPosition.getY() >= 4.035) {
                    CurrentGoalPos = blueRight;
                } else {
                    CurrentGoalPos = blueLeft;
                }
                }
                break;
        
            default:
                break;
        }
            Translation2d GoalVector = (CurrentGoalPos.minus(ShooterPosition));
            Translation2d FinalVector = GoalVector.plus(new Translation2d((TotalRobotPose.Speeds.vxMetersPerSecond)*1.5, (TotalRobotPose.Speeds.vyMetersPerSecond)*1.5));
            CalculatedDistance = Units.metersToInches(FinalVector.getNorm());
            RobotAngle = movingPose.getRotation().getDegrees();
            CalculatedAngle = FinalVector.getAngle().getDegrees()-RobotAngle+178;
            if (CalculatedAngle < TurretMin){
                CalculatedAngle = CalculatedAngle+360;
            }
            if (CalculatedAngle > TurretMax){
                CalculatedAngle = CalculatedAngle-360;
            }
            TurretAngleMotor.setControl(positionControl.withPosition(Degrees.of(CalculatedAngle*TurretGearRatio)));
            CalculatedHood = (.0855294*CalculatedDistance)-3.42198;
            if(CalculatedHood < HoodMin){
                CalculatedHood = HoodMin;
            }
            if(CalculatedHood > HoodMax){
                CalculatedHood = HoodMax;
            }
            if(HoodDown == true){
                TurretHoodMotor.setControl(positionControl.withPosition(0.0));
            }
            else {
                TurretHoodMotor.setControl(positionControl.withPosition(Degrees.of(CalculatedHood*HoodGearRatio)));
            }
            //CalculatedShooter = (0.000880253*CalculatedDistance*CalculatedDistance)-(0.0716505*CalculatedDistance)+31.81023;
            CalculatedShooter = (0.000700253*CalculatedDistance*CalculatedDistance)-(0.0706505*CalculatedDistance)+31.81023;
            if(CalculatedShooter < ShooterMin){
                CalculatedShooter = ShooterMin;
            }
            if(CalculatedShooter > ShooterMax){
                CalculatedShooter = ShooterMax;
            }
            TurretShooterMotor.setControl(velocityControl.withVelocity(CalculatedShooter*ShooterGearRatio));
        },
        ()->{
            TurretShooterMotor.setControl(velocityControl.withVelocity(0.0));
        }
        );
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
}
public enum Targets{
    blueGoal,redGoal,blueAlliance,redAlliance
}
}
