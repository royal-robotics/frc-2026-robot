package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
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
    public Turret() {
        TurretAngleMotor = new TalonFX(0);
        TurretHoodMotor = new TalonFX(0);
        TurretShooterMotor = new TalonFX(0);
        TurretShooterFollowerMotor = new TalonFX(0);

        TurretAngleMotor.getConfigurator().apply(outfitConfigs);
        TurretHoodMotor.getConfigurator().apply(outfitConfigs);
        TurretShooterMotor.getConfigurator().apply(outfitConfigs);
        TurretShooterFollowerMotor.getConfigurator().apply(outfitConfigs);

        TurretAngleMotor.getConfigurator().apply(limitsConfigs);
        TurretHoodMotor.getConfigurator().apply(limitsConfigs);
        TurretShooterMotor.getConfigurator().apply(limitsConfigs);
        TurretShooterFollowerMotor.getConfigurator().apply(limitsConfigs);

        TurretAngleMotor.getConfigurator().apply(slotConfigs);

        TurretAngleSmall = new CANcoder(0);
        TurretAngleBig = new CANcoder(0);
        TurretAngleSmall.getConfigurator().apply(magnetConfigs);
        TurretAngleBig.getConfigurator().apply(magnetConfigs);
    }
    public void periodic() {

    }

}
