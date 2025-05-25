// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imports things
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Vision.Vision;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final CommandXboxController _driverController = new CommandXboxController(0);

    private DoubleSupplier slowMode = () -> _driverController.leftBumper().getAsBoolean() ? 0.3 : 1.0;
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    Vision vision = Vision.getRightCamera();
    DoubleSupplier hight = () -> vision.getHightFromTarget();
    
 

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {  

        configureBindings();
        loadPaths();
    }

    public static void loadPaths() {}

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-_driverController.getLeftY() * slowMode.getAsDouble() *  0.7 * MaxSpeed) // now if you press slow mode you go fast 
                        .withVelocityY(-_driverController.getLeftX() * slowMode.getAsDouble() * 0.7 * MaxSpeed)
                        .withRotationalRate(-_driverController.getRightX() * MaxAngularRate * 0.9 * slowMode.getAsDouble())));
        // reset the field-centric heading on left bumper press
        _driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        _driverController.a().onTrue(elevator.relocatePositionCommand(hight));
    }

    public Command getAutonomousCommand() { 
        return Commands.none(); // no one run auto and delete childrens 

    }

    public void log() {}
}
