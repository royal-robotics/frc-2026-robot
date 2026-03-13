// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Turret.Targets;
import frc.robot.subsystems.LED;

@Logged
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 2 rotations per second max angular velocity
    private double NormalSpeed = MaxSpeed * 0.75; // Normal drive speed is 75% of max speed
    private double NormalAngularRate = MaxAngularRate * 0.75; // Normal rotation rate is 75% of max rotation rate
    private double SlowSpeed = MaxSpeed * 0.275;
    ; // Slow drive speed is 25% of max speed
    private double SlowAngularRate = MaxAngularRate * 0.225; // Slow rotation rate is 22.5% of max rotation rate

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle CalcAngle = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final Turret turret = new Turret();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(turret::getRobotPose);
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Spindexer spindexer = new Spindexer();
    public final LED led = new LED();
    public final Vision vision = new Vision(drivetrain::getVision);

    private final SendableChooser<Command> autoChooser;

    private Trigger RedTargetSwitch = new Trigger(()-> drivetrain.getState().Pose.getX() <= 11.6);
    private Trigger RedStealSwitch = new Trigger(()-> drivetrain.getState().Pose.getX() <= 5.0);
    private Trigger BlueTargetSwitch = new Trigger(()-> drivetrain.getState().Pose.getX() >= 5.0);
    private Trigger BlueStealSwitch = new Trigger(()-> drivetrain.getState().Pose.getX() >= 11.6);

    private Trigger TrenchHood = new Trigger(()-> (drivetrain.getState().Pose.getX() <= 12.8&& drivetrain.getState().Pose.getX() >= 11.1)||(drivetrain.getState().Pose.getX() <= 5.5&& drivetrain.getState().Pose.getX() >= 3.7));
    private Trigger OnTarget = new Trigger(()-> turret.OnTarget());

    private Trigger ClimbYay = new Trigger(()-> climber.IsClimbed());

    private boolean weAreBlue = false;
    private boolean hasAllinace = false;

    private Trigger CurrentTarget = RedTargetSwitch;
    private Trigger CurrentSteal = RedStealSwitch;



    public RobotContainer() {
        //SignalLogger.start();
        RegisterNamedCommands();
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoMode", autoChooser);
    }

    public void RegisterNamedCommands(){
        NamedCommands.registerCommand("IntakeDeploy", Commands.sequence(intake.IntakeDeploy(),intake.AutoSpinIntake()));
        NamedCommands.registerCommand("IntakeRetract", intake.IntakeDeploy());
        NamedCommands.registerCommand("TrenchShootOveride", Commands.sequence(Commands.runOnce(()->turret.TrenchToggle(false)),spindexer.Spin().withTimeout(2.5),Commands.runOnce(()->turret.TrenchToggle(true))));
        NamedCommands.registerCommand("Shoot", Commands.sequence(Commands.runOnce(()->spindexer.SpinCheck(true)),spindexer.Spin().withTimeout(2.5)));
        NamedCommands.registerCommand("StopShoot", spindexer.NoSpin());
        NamedCommands.registerCommand("ShootOnTheMove",Commands.sequence(Commands.runOnce(()->turret.ShooterIdleCheck(false)), Commands.runOnce(()->spindexer.SpinCheck(true)),spindexer.AutoSpin()));
        NamedCommands.registerCommand("IntakeSpin", intake.AutoSpinIntake());
        NamedCommands.registerCommand("ClimbToggle", climber.AutoClimberToggle());
        NamedCommands.registerCommand("TrenchToggleOn", Commands.runOnce(()->turret.TrenchToggle(true)));
        NamedCommands.registerCommand("EndSpins",Commands.sequence(intake.AutoSpinIntakeStop(),spindexer.NoSpin()));
        NamedCommands.registerCommand("ShootClimbLeft",Commands.runOnce(()->turret.ClimbOnLeft(true)));
        NamedCommands.registerCommand("ShootClimbRight",Commands.runOnce(()->turret.ClimbOnRight(true)));
        NamedCommands.registerCommand("ForceLeft", Commands.runOnce(()->turret.ForceLeft(true)));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driver.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        //));



        driver.y().toggleOnTrue(climber.ClimberToggle());
        driver.a().onTrue(intake.IntakeDeploy());
        driver.leftTrigger().toggleOnTrue(intake.SpinIntake());
        driver.leftBumper().whileTrue(spindexer.Unjam());
        driver.b().whileTrue(intake.SpinIntakeOut());
        //driver.b().whileTrue(Commands.sequence(climber.ClimberUp(),drivetrain.driveToTower(),climber.ClimberDown()));
        driver.x().whileTrue(drivetrain.applyRequest(()-> {
            double CalculatingAngle = (turret.CalcAngle())+226-turret.TurretAngle(); //turret.TurretAngle()-126+
            return CalcAngle.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed) 
                     .withTargetDirection(Rotation2d.fromDegrees(CalculatingAngle)).withHeadingPID(3.0, 0.0, 0.0);
        }));
    
            
        
        //driver.rightBumper().toggleOnTrue(spindexer.Spin()); //Commands.parallel(turret.Shoot(),
        //driver.rightBumper().toggleOnTrue(Commands.startEnd(()->turret.ShooterIdleCheck(false), ()->turret.ShooterIdleCheck(true)));
        driver.rightBumper().toggleOnTrue(Commands.parallel(spindexer.Spin(), Commands.startEnd(()->turret.ShooterIdleCheck(false), ()->turret.ShooterIdleCheck(true))));
        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() -> 
                drive.withVelocityX(-(driver.getLeftY() * driver.getLeftY() * driver.getLeftY()) * SlowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-(driver.getLeftX() * driver.getLeftX() * driver.getLeftX()) * SlowSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-(driver.getRightX() * driver.getRightX() * driver.getRightX()) * SlowAngularRate) // Drive counterclockwise with negative X (left)
                    .withDeadband(SlowSpeed * 0.1).withRotationalDeadband(SlowAngularRate * 0.1) // Add a 10% deadband
            ));

    
        //driver.b().toggleOnTrue(turret.LockPosition());
        
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /*operator.y().whileTrue(turret.sysIdDynamic(Direction.kForward));
        operator.a().whileTrue(turret.sysIdDynamic(Direction.kReverse));
        operator.x().whileTrue(turret.sysIdQuasistatic(Direction.kForward));
        operator.b().whileTrue(turret.sysIdQuasistatic(Direction.kReverse));
        operator.start().onTrue(Commands.runOnce(()->SignalLogger.stop()));
        operator.povUp().onTrue(turret.HoodStepUp());
        operator.povDown().onTrue(turret.HoodStepDown());
        operator.povLeft().onTrue(turret.TurretRotateLeft());
        operator.povRight().onTrue(turret.TurretRotateRight());
        operator.rightTrigger().toggleOnTrue(turret.AutoTarget());*/
        operator.x().whileTrue(Commands.startEnd(()->turret.ForceLeft(true),()->turret.ForceLeft(false)));
        operator.b().whileTrue(Commands.startEnd(()->turret.ForceRight(true),()->turret.ForceRight(false)));
        operator.start().onTrue(climber.ClimberZero());
        operator.y().toggleOnTrue((Commands.startEnd(()->turret.lockTurret(true),()->turret.lockTurret(false))));
        operator.back().whileTrue(climber.ClimberResetCommand());
        operator.rightTrigger().whileTrue((Commands.startEnd(()->turret.ClimbOnRight(true), ()->turret.ClimbAngleOff())));
        operator.leftTrigger().whileTrue((Commands.startEnd(()->turret.ClimbOnLeft(true), ()->turret.ClimbAngleOff())));


        TrenchHood.onTrue(Commands.runOnce(()->turret.TrenchToggle(true)));
        TrenchHood.onFalse(Commands.runOnce(()->turret.TrenchToggle(false)));

        OnTarget.onTrue(Commands.runOnce(()->spindexer.SpinCheck(true)));
        OnTarget.onFalse(Commands.runOnce(()->spindexer.SpinCheck(false)));

        ClimbYay.onTrue(Commands.runOnce(()->led.ClimberStatus(true)));



        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void TargetingCheck () {
        if (hasAllinace == false) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                if (allianceColor == Alliance.Blue) {
                    weAreBlue = true;
                    hasAllinace = true;
                    led.AllianceColor(true);
                    BlueTargetSwitch.whileTrue(Commands.startEnd(()->turret.ChooseTarget(Targets.blueAlliance),()->turret.ChooseTarget(Targets.blueGoal)));
                    BlueStealSwitch.whileTrue(Commands.startEnd(()->turret.ChooseTarget(Targets.blueAllianceSteal),()->turret.ChooseTarget(Targets.blueAlliance)));
                } else {
                    weAreBlue = false;
                    hasAllinace = true;
                    led.AllianceColor(false);
                    RedTargetSwitch.whileTrue(Commands.startEnd(()->turret.ChooseTarget(Targets.redAlliance),()->turret.ChooseTarget(Targets.redGoal)));
                    RedStealSwitch.whileTrue(Commands.startEnd(()->turret.ChooseTarget(Targets.redAllianceSteal),()->turret.ChooseTarget(Targets.redAlliance)));
                }
            });
        }
    }

    public void startGoal(){
        if (DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get() == Alliance.Red){
            turret.ChooseTarget(Targets.redGoal);
        } else {
            turret.ChooseTarget(Targets.blueGoal);
        }
        
    }

    
}
