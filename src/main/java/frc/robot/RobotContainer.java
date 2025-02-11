// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//imports things
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.CommandGroupFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Gripper.GripperSubsystem;
import frc.robot.subsystems.gripperArm.GripperArm;
import frc.robot.subsystems.gripperArm.GripperArmConstants;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private DoubleSupplier targetAngle = _driverController.povUp().getAsBoolean() ? () -> 0.0 :
                                      _driverController.povRight().getAsBoolean() ? () -> 90.0 :
                                      _driverController.povDown().getAsBoolean() ? () -> 180.0 : 
                                      _driverController.povLeft().getAsBoolean() ? () -> 270.0 : () -> -1;
                                      
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle angleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private final GripperSubsystem gripper = GripperSubsystem.getInstance();
    private final GripperArm gripperArm = GripperArm.getInstance();


    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final CommandXboxController _driverController = new CommandXboxController(0);
    public static final CommandXboxController _operatorController = new CommandXboxController(1);
    public static final CommandXboxController _testerController = new CommandXboxController(2);
    public static final CommandXboxController _sysIdController = new CommandXboxController(3);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-_testerController.getLeftY() * MaxSpeed * 0.1) // Drive forward with negative Y (forward)
                    .withVelocityY(-_testerController.getLeftX() * MaxSpeed * 0.1) // Drive left with negative X (left)
                    .withRotationalRate(-_testerController.getRightX() * MaxAngularRate * 0.1) // Drive counterclockwise with negative X (left)
            ).onlyWhile(() -> targetAngle.getAsDouble() == -1)
        );

        //Driver Controller
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                angleDrive.withVelocityX(-_testerController.getLeftY() * MaxSpeed * 0.1) // Drive forward with negative Y (forward)
                    .withVelocityY(-_testerController.getLeftX() * MaxSpeed * 0.1) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d((Math.toRadians(targetAngle.getAsDouble())))) // Drive counterclockwise with negative X (left)
                    ).unless(() -> targetAngle.getAsDouble() == -1)
        );

        //Operator Controller

        //cannon
        _operatorController.a().onTrue(cannon.adjustCoralCommand());
        _operatorController.y().onTrue(cannon.loosenCoralCommand());

        //gripper arm
        gripperArm.setDefaultCommand(gripperArm.relocateAngelCommand(_operatorController));

        //elevator buttons
        elevator.setDefaultCommand(elevator.relocatePositionCommand());
        _operatorController.povUp().onTrue(elevator.setTargetPositionCommand(ElevatorConstanst.L1_HEIGHT));
        _operatorController.povRight().onTrue(elevator.setTargetPositionCommand(ElevatorConstanst.L2_HEIGHT));
        _operatorController.povDown().onTrue(elevator.setTargetPositionCommand(ElevatorConstanst.L3_HEIGHT));
        _operatorController.povLeft().onTrue(elevator.setTargetPositionCommand(ElevatorConstanst.L4_HEIGHT)); 

        //gripper (right/left joysticks)
        _operatorController.leftBumper().onTrue(gripper.collectUntilCollectedCommand());
        _operatorController.rightBumper().onTrue(gripper.tossCommand());

        //resets
        _testerController.back().onTrue(gripperArm.setHomeCommand());
        _operatorController.start().onTrue(elevator.resetElevatorCommand());


        //Tester
        _testerController.a().onTrue(cannon.adjustCoralCommand());
        _testerController.y().onTrue(cannon.loosenCoralCommand());
        _testerController.leftBumper().onTrue(gripper.collectUntilCollectedCommand());
        _testerController.rightBumper().onTrue(gripper.tossCommand());

        _testerController.povUp().whileTrue(elevator.moveCommand(1));
        _testerController.povDown().whileTrue(elevator.moveCommand(-1));
        _testerController.start().onTrue(elevator.resetElevatorCommand());

        
        _testerController.povRight().whileTrue(gripperArm.moveArmCommand(-1));
        _testerController.povLeft().whileTrue(gripperArm.moveArmCommand(1));

        _testerController.rightTrigger().onTrue(CommandGroupFactory.putCoralTakeAlgea(ElevatorConstanst.L3_HEIGHT,GripperArmConstants.REEF_ANGLE));

        //sysysysy
        _sysIdController.back().and(_sysIdController.y()).whileTrue(gripperArm.sysIdDynamic(Direction.kForward));
        _sysIdController.back().and(_sysIdController.x()).whileTrue(gripperArm.sysIdDynamic(Direction.kReverse));
        _sysIdController.start().and(_sysIdController.y()).whileTrue(gripperArm.sysIdQuasistatic(Direction.kForward));
        _sysIdController.start().and(_sysIdController.x()).whileTrue(gripperArm.sysIdQuasistatic(Direction.kReverse));
        _sysIdController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        _sysIdController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // _driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // _driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-_driverController.getLeftY(), -_driverController.getLeftX()))
        // ));

        // _driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // _driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
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
        _driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected(); //todo: connect the chooser to the path
    }
}
