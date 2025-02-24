package frc.robot.PrimoLib;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.CommandGroupFactory;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Vision.Vision;

public class Elastic {
    static Field2d m_field = new Field2d();
    public static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public static CannonSubsystem cannonSubsystem = CannonSubsystem.getInstance();

    public static HashMap<String, PathPlannerPath> pathsMap = new HashMap<>();
    public static CommandSelector commandSelector;
    static final String[] PATHS = new String[]{
        // "H path",
        "H to collection",
        // "Collection2 to D",
        // "literallyWalkForward"
    };

    public static void loadPaths() {
        for(String path : PATHS) {
            try {
                pathsMap.put(path, PathPlannerPath.fromPathFile(path));
                System.out.println("LOADED PROPERLY!!!");
                System.out.println("LOADED PROPERLY!!!");
                System.out.println("LOADED PROPERLY!!!");
                System.out.println("LOADED PROPERLY!!!");
                System.out.println("LOADED PROPERLY!!!");
                System.out.println("LOADED PROPERLY!!!");
                System.out.println("LOADED PROPERLY!!!");
            } catch (Exception e) {
                System.out.println(path + " didn't load properly!!!1");
                System.out.println(path + " didn't load properly!!!2");
                System.out.println(path + " didn't load properly!!!3");
                System.out.println(path + " didn't load properly!!!4");
                System.out.println(path + " didn't load properly!!!5");
                // System.out.println(e.getMessage());
                e.printStackTrace();
                // TODO: handle exception
            }
        }
    }


    public static void autoSelector(){
    //Creating SendableChooser Object

    SendableChooser m_chooserIntakeStaion = new SendableChooser<>();
    SendableChooser m_chooserGetOutTheWay = new SendableChooser<>();

    SendableChooser m_chooserReefCycle1 = new SendableChooser<>();
    SendableChooser m_chooserLevelCycle1 = new SendableChooser<>();

    SendableChooser m_chooserReefCycle2 = new SendableChooser<>();
    SendableChooser m_chooserLevelCycle2 = new SendableChooser<>();

    SendableChooser m_chooserReefCycle3 = new SendableChooser<>();
    SendableChooser m_chooserLevelCycle3 = new SendableChooser<>();

    m_chooserIntakeStaion.setDefaultOption("right", "right");
    m_chooserIntakeStaion.addOption("right", "right");
    m_chooserIntakeStaion.addOption("left", "left");
    SmartDashboard.putData("Intake staion", m_chooserIntakeStaion);

    m_chooserGetOutTheWay.setDefaultOption("false", "false");
    m_chooserGetOutTheWay.addOption("false", "false");
    m_chooserGetOutTheWay.addOption("true", "true");
    SmartDashboard.putData("GetOutTheWay?", m_chooserGetOutTheWay);

    SequentialCommandGroup coralH4_Collect_D = 
        // AutoBuilder.followPath(pathsMap.get("H path"))
        new InstantCommand()
        .andThen(CommandGroupFactory.putCoral(ElevatorConstanst.L4_HEIGHT))
        // .andThen(AutoBuilder.followPath(pathsMap.get("H to collection")))
        
        .andThen(cannonSubsystem.collectUntilCoralCommand().withTimeout(1));
        // .andThen(AutoBuilder.followPath(pathsMap.get("Collection2 to D")));
        
    Command literallyWalkForward = new InstantCommand();
        // AutoBuilder.followPath(pathsMap.get("literallyWalkForward"));
        

    HashMap<String, Command> commands = new HashMap<>();
    commands.put("literallyWalkForwards", literallyWalkForward);
    commands.put("Coral H4, Collect, Go To D", coralH4_Collect_D);
    commandSelector = new CommandSelector(commands, "Auto Chooser yay");
    
    // ^cycle 1
    m_chooserReefCycle1.setDefaultOption("Reef-A", "Reef-A"); //the one selected by default when the dashboard starts
    m_chooserReefCycle1.addOption("Reef-A", "Reef-A");
    m_chooserReefCycle1.addOption("Reef-B", "Reef-B");
    m_chooserReefCycle1.addOption("Reef-C", "Reef-C");
    m_chooserReefCycle1.addOption("Reef-D", "Reef-D");
    m_chooserReefCycle1.addOption("Reef-E", "Reef-E");
    m_chooserReefCycle1.addOption("Reef-F", "Reef-F");
    m_chooserReefCycle1.addOption("Reef-G", "Reef-G");
    m_chooserReefCycle1.addOption("Reef-H", "Reef-H");
    m_chooserReefCycle1.addOption("Reef-I", "Reef-I");
    m_chooserReefCycle1.addOption("Reef-J", "Reef-J");
    m_chooserReefCycle1.addOption("Reef-K", "Reef-K");
    m_chooserReefCycle1.addOption("Reef-L", "Reef-L");

    SmartDashboard.putData("cycle 1 reef", m_chooserReefCycle1); //putting on dashboard

    m_chooserLevelCycle1.setDefaultOption("L4", "L4");
    m_chooserLevelCycle1.addOption("L1", "L1");
    m_chooserLevelCycle1.addOption("L2", "L2");
    m_chooserLevelCycle1.addOption("L3", "L3");
    m_chooserLevelCycle1.addOption("L4", "L4");

    SmartDashboard.putData("cycle 1 level", m_chooserLevelCycle1);

    // ^cycle 2
    m_chooserReefCycle2.setDefaultOption("Reef-A", "Reef-A"); //the one selected by default when the dashboard starts
    m_chooserReefCycle2.addOption("Reef-A", "Reef-A");
    m_chooserReefCycle2.addOption("Reef-B", "Reef-B");
    m_chooserReefCycle2.addOption("Reef-C", "Reef-C");
    m_chooserReefCycle2.addOption("Reef-D", "Reef-D");
    m_chooserReefCycle2.addOption("Reef-E", "Reef-E");
    m_chooserReefCycle2.addOption("Reef-F", "Reef-F");
    m_chooserReefCycle2.addOption("Reef-G", "Reef-G");
    m_chooserReefCycle2.addOption("Reef-H", "Reef-H");
    m_chooserReefCycle2.addOption("Reef-I", "Reef-I");
    m_chooserReefCycle2.addOption("Reef-J", "Reef-J");
    m_chooserReefCycle2.addOption("Reef-K", "Reef-K");
    m_chooserReefCycle2.addOption("Reef-L", "Reef-L");

    SmartDashboard.putData("cycle 2 reef", m_chooserReefCycle2); //putting on dashboard
    m_chooserLevelCycle2.setDefaultOption("L4", "L4");
    m_chooserLevelCycle2.addOption("L1", "L1");
    m_chooserLevelCycle2.addOption("L2", "L2");
    m_chooserLevelCycle2.addOption("L3", "L3");
    m_chooserLevelCycle2.addOption("L4", "L4");

    SmartDashboard.putData("cycle 2 level", m_chooserLevelCycle2);

    // ^cycle 3
    m_chooserReefCycle3.setDefaultOption("Reef-A", "Reef-A"); //the one selected by default when the dashboard starts
    m_chooserReefCycle3.addOption("Reef-A", "Reef-A");
    m_chooserReefCycle3.addOption("Reef-B", "Reef-B");
    m_chooserReefCycle3.addOption("Reef-C", "Reef-C");
    m_chooserReefCycle3.addOption("Reef-D", "Reef-D");
    m_chooserReefCycle3.addOption("Reef-E", "Reef-E");
    m_chooserReefCycle3.addOption("Reef-F", "Reef-F");
    m_chooserReefCycle3.addOption("Reef-G", "Reef-G");
    m_chooserReefCycle3.addOption("Reef-H", "Reef-H");
    m_chooserReefCycle3.addOption("Reef-I", "Reef-I");
    m_chooserReefCycle3.addOption("Reef-J", "Reef-J");
    m_chooserReefCycle3.addOption("Reef-K", "Reef-K");
    m_chooserReefCycle3.addOption("Reef-L", "Reef-L");

    SmartDashboard.putData("cycle 3 reef", m_chooserReefCycle3); //putting on dashboard

    m_chooserLevelCycle3.setDefaultOption("L4", "L4");
    m_chooserLevelCycle3.addOption("L1", "L1");
    m_chooserLevelCycle3.addOption("L2", "L2");
    m_chooserLevelCycle3.addOption("L3", "L3");
    m_chooserLevelCycle3.addOption("L4", "L4");

    SmartDashboard.putData("cycle 3 level", m_chooserLevelCycle2);


    }

    public static void dispalyCommandScheduler(){
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance() );
    }

    public static void displayCameraDeta(){
        Vision camera = Vision.getFrontCamera();
        SmartDashboard.putNumber("Vision/front/angelFromTarget", camera.getAngleFromTarget());
    }

    public static void displayField(){
        // Do this in either robot or subsystem init
        SmartDashboard.putData("Field", m_field);
    }

    public static void displayRobotPose(){
        m_field.setRobotPose(RobotContainer.drivetrain.getState().Pose);
    }

    public static void setTargetPose(Pose2d targetPose) {
        m_field.getObject("Reef Target").setPose(targetPose);
    }
}
