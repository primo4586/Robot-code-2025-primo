// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imports things
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Auto.AutoCommands;
import frc.robot.Commands.swerveCommands.driveToPointWithPIDCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    SlewRateLimiter xAccLimiterb = new SlewRateLimiter(10);
    SlewRateLimiter yAccLimiterb = new SlewRateLimiter(10);
    SlewRateLimiter rotAccLimiterb = new SlewRateLimiter(10);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final CommandXboxController _driverController = new CommandXboxController(0);
    public static final CommandXboxController _operatorController = new CommandXboxController(1);
    // public static final CommandXboxController _testerController = new CommandXboxController(2);
    // public static final CommandXboxController _sysIdController = new CommandXboxController(3);

    private DoubleSupplier slowMode = () -> _driverController.leftBumper().getAsBoolean() ? 0.3 : 1.0;

    private DoubleSupplier targetAngle = () -> _driverController.povUp().getAsBoolean() ? 0.0
            : _driverController.povRight().getAsBoolean() ? 90.0
                    : _driverController.povDown().getAsBoolean() ? 180.0
                            : _driverController.povLeft().getAsBoolean() ? 270.0 : -1;

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("putCoralL4", AutoCommands.putCoralL4());
        NamedCommands.registerCommand("putCoralL3", AutoCommands.putCoralL3());
        NamedCommands.registerCommand("putCoralL2", AutoCommands.putCoralL2());
        NamedCommands.registerCommand("putCoralL1", AutoCommands.putCoralL1());

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // // Note that X is defined as forward according to WPILib convention,
        // // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        // new ConditionalCommand(
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(_driverController.getLeftY() * slowMode.getAsDouble() *
        // 0.45 * MaxSpeed)
        // .withVelocityY(_driverController.getLeftX() * slowMode.getAsDouble() * 0.45 *
        // MaxSpeed)
        // .withRotationalRate(_driverController.getRightX() * MaxAngularRate * 0.9)),
        // drivetrain.applyRequest(() ->
        // angleDrive.withVelocityX(_driverController.getLeftY() * MaxSpeed * 0.45) //
        // Drive forward with negative Y (forward)
        // .withVelocityY(_driverController.getLeftX() * MaxSpeed * 0.45) // Drive left
        // with negative X (left)
        // .withTargetDirection(new
        // Rotation2d((Math.toRadians(targetAngle.getAsDouble()))))),
        // () -> targetAngle.getAsDouble() == -1
        // )
        // );
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-_driverController.getLeftY() * slowMode.getAsDouble() * 0.7 * MaxSpeed)
                        .withVelocityY(-_driverController.getLeftX() * slowMode.getAsDouble() * 0.7 * MaxSpeed)
                        .withRotationalRate(-_driverController.getRightX() * MaxAngularRate * 0.9)));

        // //driver Controller
        // drivetrain.setDefaultCommand(
        // // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() ->
        // angleDrive.withVelocityX(-_driverController.getLeftY() * MaxSpeed * 0.1) //
        // Drive forward with negative Y (forward)
        // .withVelocityY(-_driverController.getLeftX() * MaxSpeed * 0.1) // Drive left
        // with negative X (left)
        // .withTargetDirection(new
        // Rotation2d((Math.toRadians(targetAngle.getAsDouble())))) // Drive
        // counterclockwise with negative X (left)
        // ).unless(() -> targetAngle.getAsDouble() == -1)
        // );

        // temp
        elevator.setDefaultCommand(elevator.relocatePositionCommand());
        _operatorController.x().whileTrue(elevator.moveCommand(1).andThen(elevator.relocatePositionCommand()));
        ;
        _operatorController.b().whileTrue(elevator.moveCommand(-1).andThen(elevator.relocatePositionCommand()));
        _driverController.leftStick().onTrue(cannon.loosenCoralCommand());
        _driverController.rightStick().onTrue(cannon.loosenCoralCommand());
        
        

        _driverController.leftTrigger().whileTrue(new driveToPointWithPIDCommand(false));
        _driverController.rightTrigger().whileTrue(new driveToPointWithPIDCommand(true));

        // Operator Controller

        // cannon
        _operatorController.a().onTrue(cannon.adjustCoralCommand());
        _operatorController.y().onTrue(cannon.loosenCoralCommand());
        _operatorController.start().onTrue(cannon.stopMotorCommand());

        // elevator buttons

        _operatorController.povUp().onTrue(elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT));
        _operatorController.povRight().onTrue(elevator.relocatePositionCommand(ElevatorConstanst.L2_HEIGHT));
        _operatorController.povDown().onTrue(elevator.relocatePositionCommand(ElevatorConstanst.L3_HEIGHT));
        _operatorController.povLeft().onTrue(elevator.relocatePositionCommand(ElevatorConstanst.L4_HEIGHT));


        // // Tester
        // _testerController.a().onTrue(cannon.catchCoralCommand());
        // _testerController.y().onTrue(cannon.loosenCoralCommand());

        // _testerController.povUp().whileTrue(elevator.moveCommand(1));
        // _testerController.povDown().whileTrue(elevator.moveCommand(-1));
        // _testerController.start().onTrue(elevator.resetElevatorCommand());

        // _sysIdController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // _sysIdController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // _driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // _driverController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-_driverController.getLeftY(),
        // -_driverController.getLeftX()))
        // ));

        // _driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
        // forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // _driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
        // forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // _driverController.back().and(_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // _driverController.back().and(_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // _driverController.start().and(_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // _driverController.start().and(_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // m_joystick.y().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // m_joystick.a().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // m_joystick.b().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // m_joystick.x().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // reset the field-centric heading on left bumper press
        _driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return AutoCommands.normalCommand(); // TODO: connect the chooser to the path

    }

    public void log() {
        SmartDashboard.putNumber("slowMode", slowMode.getAsDouble());
        SmartDashboard.putNumber("swerve angel ", targetAngle.getAsDouble());
    }
}
