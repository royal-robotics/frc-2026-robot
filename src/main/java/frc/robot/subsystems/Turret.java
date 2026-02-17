package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private CANBus canBus = new CANBus("CAN");

    private TalonFX TurretAngleMotor;
    private TalonFX TurretHoodMotor;
    private TalonFX TurretShooterMotor;
    private TalonFX TurretShooterFollowerMotor;

    private CANcoder TurretAngleSmall;
    private CANcoder TurretAngleBig;

    private MotorOutputConfigs outfitConfigs = new MotorOutputConfigs();
    private CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
    private Slot0Configs slotConfigs = new Slot0Configs();
    private MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();

    private StatusSignal<Angle> turretAngleSignal;
    private StatusSignal<Angle> turretHoodSignal;
    private StatusSignal<AngularVelocity> turretShooterSignal;

    private PositionVoltage positionControl = new PositionVoltage(Rotations.of(0.0));
    private VelocityVoltage velocityControl = new VelocityVoltage(RotationsPerSecond.of(0.0));
    private Follower followControl = new Follower(11, MotorAlignmentValue.Aligned);

    private boolean TurretOverride = false;
    private double TurretAngleOverride = 0.0;
    private double TurretHoodOverride = 0.0;
    private double TurretShooterOverride = 0.0;

    private double TurretGearRatio = 46;
    private double HoodGearRatio = (40.0/15.0)*(30.0/18.0)*(197.0/10.0);
    private double ShooterGearRatio = (32.0/16.0)*(16.0/15.0);

    public Turret() {
        TurretAngleMotor = new TalonFX(13, canBus);
        TurretHoodMotor = new TalonFX(10, canBus);
        TurretShooterMotor = new TalonFX(11, canBus);
        TurretShooterFollowerMotor = new TalonFX(12,canBus);

        TurretAngleMotor.getConfigurator().apply(outfitConfigs);
        TurretHoodMotor.getConfigurator().apply(outfitConfigs);
        TurretShooterMotor.getConfigurator().apply(outfitConfigs);
        TurretShooterFollowerMotor.getConfigurator().apply(outfitConfigs);

        TurretAngleMotor.getConfigurator().apply(limitsConfigs);
        TurretHoodMotor.getConfigurator().apply(limitsConfigs);
        TurretShooterMotor.getConfigurator().apply(limitsConfigs);
        TurretShooterFollowerMotor.getConfigurator().apply(limitsConfigs);

        TurretAngleMotor.getConfigurator().apply(slotConfigs);

        TurretAngleSmall = new CANcoder(5,canBus);
        TurretAngleBig = new CANcoder(6,canBus);
        TurretAngleSmall.getConfigurator().apply(magnetConfigs);
        TurretAngleBig.getConfigurator().apply(magnetConfigs);

        turretAngleSignal = TurretAngleMotor.getPosition();
        turretHoodSignal = TurretHoodMotor.getPosition();
        turretShooterSignal = TurretShooterMotor.getVelocity();

        turretShooterSignal.waitForUpdate(0.02);
        TurretShooterFollowerMotor.setControl(followControl);

        SmartDashboard.putBoolean("TurretManualOverride", TurretOverride);
        SmartDashboard.putNumber("TurretAngleOverride", TurretAngleOverride);
        SmartDashboard.putNumber("TurretHoodOverride", TurretHoodOverride);
        SmartDashboard.putNumber("TurretShooterOverride", TurretShooterOverride);
    }

    public double TurretAngle() {
        return turretAngleSignal.getValueAsDouble();
    }

    public double TurretHood() {
        return turretHoodSignal.getValueAsDouble();
    }

    public double TurretShooter() {
        return turretShooterSignal.getValueAsDouble();
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(turretAngleSignal, turretHoodSignal, turretShooterSignal);
    }


  public Command TurretManual(){
      return run(()->{
        if (SmartDashboard.getBoolean("TurretManualOverride", TurretOverride)){
            TurretAngleMotor.setControl(positionControl.withPosition(SmartDashboard.getNumber("TurretAngleOverride", TurretAngleOverride)*TurretGearRatio));
            TurretHoodMotor.setControl(positionControl.withPosition(SmartDashboard.getNumber("TurretHoodOverride", TurretHoodOverride)*HoodGearRatio));
            TurretShooterMotor.setControl(velocityControl.withVelocity(SmartDashboard.getNumber("TurretShooterOverride", TurretShooterOverride)*ShooterGearRatio));
         }

        else {
            TurretAngleOverride = TurretAngle();
            TurretHoodOverride = TurretHood();
            TurretShooterOverride = TurretShooter();
            SmartDashboard.putNumber("TurretAngleOverride", TurretAngleOverride);
            SmartDashboard.putNumber("TurretHoodOverride", TurretHoodOverride);
            SmartDashboard.putNumber("TurretShooterOverride", TurretShooterOverride);
         }
      });
  }
}
