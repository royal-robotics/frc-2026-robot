package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private TalonFX TurretAngleMotor;
    private TalonFX TurretHoodMotor;
    private TalonFX TurretShooterMotor;
    private TalonFX TurretShooterFollowerMotor;

    private CANcoder TurrentAngleSmall;
    private CANcoder TurrentAngleBig;
    private StatusSignal TurretAngleSignal;

    public Turret() {
        TurretAngleMotor = new TalonFX(0);
        TurretHoodMotor = new TalonFX(0);
        TurretShooterMotor = new TalonFX(0);
        TurretShooterFollowerMotor = new TalonFX(0);

        TurrentAngleSmall = new CANcoder(0);
        TurrentAngleBig = new CANcoder(0);
        TurretAngleSignal = TurretAngleMotor.getPosition();
    }
    public void periodic() {

    }

}
