// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imports things
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CommandGroupFactory;
import frc.robot.Commands.Auto.AutoCommands;
import frc.robot.Commands.swerveCommands.DriveToDistanceWithCamera;
import frc.robot.PrimoLib.Elastic;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Disposer.Disposer;
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
    private final Disposer disposer = Disposer.getInstance();

    SlewRateLimiter xAccLimiterb = new SlewRateLimiter(10);
    SlewRateLimiter yAccLimiterb = new SlewRateLimiter(10);
    SlewRateLimiter rotAccLimiterb = new SlewRateLimiter(10);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final CommandXboxController _driverController = new CommandXboxController(0);
    public static final CommandXboxController _operatorController = new CommandXboxController(1);
    // public static final CommandXboxController _testerController = new CommandXboxController(2);
    // public static final CommandXboxController _sysIdController = new CommandXboxController(3);

    static final String[] PATHS = { "F to collection", "R2 to J", "J to collection", "collection to L", "L to collection", "Collection1 to K", "R3 to E", "E to collection", "collection to D", "D to collection", "collection to C"};
    // "H path",
    //"H to collection"
    // "Collection2 to D",
    // "literallyWalkForward"
    public static HashMap<String, PathPlannerPath> pathsMap = new HashMap<>();
 

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("putCoralL4", AutoCommands.putCoralL4());
        NamedCommands.registerCommand("putCoralL3", AutoCommands.putCoralL3());
        NamedCommands.registerCommand("putCoralL2", AutoCommands.putCoralL2());
        NamedCommands.registerCommand("putCoralL1", AutoCommands.putCoralL1());

        NamedCommands.registerCommand("waitToCoral", AutoCommands.waitToCoral());

        NamedCommands.registerCommand("alignToRight", new DriveToDistanceWithCamera(true));
        NamedCommands.registerCommand("alignToLeft", new DriveToDistanceWithCamera(false));

        NamedCommands.registerCommand("lowerElevator", elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT)); 
        NamedCommands.registerCommand("elevatorToL3", elevator.relocatePositionCommand(ElevatorConstanst.L3_HEIGHT));
        NamedCommands.registerCommand("elevatorToL4", elevator.relocatePositionCommand(ElevatorConstanst.L4_HEIGHT));
        NamedCommands.registerCommand("elevatroToL2", elevator.relocatePositionCommand(ElevatorConstanst.L2_HEIGHT));  

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        loadPaths();
    }

    public static void loadPaths() {
        for(String path : PATHS) {
            try {
                pathsMap.put(path, PathPlannerPath.fromPathFile(path));
                System.out.println("Loaded \"" + path + "\" successfully!");
 
            } catch (Exception e) {
                System.out.println("wasn't able to load path \"" + path + "\"");
                e.printStackTrace();
            }
        }
        PathfindingCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-_driverController.getLeftY() * 0.3 *  0.7 * MaxSpeed) // the * 0.3 is safe mode and replace slow mode 
                        .withVelocityY(-_driverController.getLeftX() * 0.3 * 0.7 * MaxSpeed)
                        .withRotationalRate(-_driverController.getRightX() * MaxAngularRate * 0.9)));
        // reset the field-centric heading on left bumper press
        _driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        _driverController.rightTrigger().onTrue(CommandGroupFactory.safePlaceCoral(true));
        _driverController.leftTrigger().onTrue(CommandGroupFactory.safePlaceCoral(false));
        _driverController.y().onTrue(CommandGroupFactory.removeAlgeaFromL2());
        _driverController.start().onTrue(disposer.goHomeCommand()); // in case it gets stuck or something
    }

    public Command getAutonomousCommand() {
        String selected = Elastic.auto.getSelected().toString();
        if(selected == "middle") {
            return AutoCommands.normalCommand();
        } else if(selected == "left") { 
            return new PathPlannerAuto("Left auto");
        } else {            
            return new PathPlannerAuto("Right auto");
        }

    }

    public void log() {

        }
}
