package frc.robot.PrimoLib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Elastic {
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
}
