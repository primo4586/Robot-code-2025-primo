// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.PrimoLib.Elastic;
import frc.robot.PrimoLib.positionCommands.UpdateGlobalPoseVision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Elastic.loadPaths();
    

    Elastic.simpleAutoSelector();
    
    Elastic.displayField();
    RobotContainer.drivetrain.setStateStdDevs(VisionConstants.kSingleTagStdDevs);
    
    // UsbCamera usbCamera = new UsbCamera("coralCam", 0);
    // usbCamera.setPixelFormat(PixelFormat.kYUYV);
    // CameraServer.startAutomaticCapture(usbCamera);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Elastic.dispalyCommandScheduler();
    Elastic.displayRobotPose();
    Elastic.displayTimer();
    m_robotContainer.log();
    UpdateGlobalPoseVision.updateGlobalPoseVision(isDisabled()); // update global pose with vision
  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    /*
     * this is to log the swerve fast and Talons infromation
     */
    SignalLogger.start();
    
  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //see which auto was selected

    // m_autoSelectedLevel = m_chooserReef.getSelected();
    // System.out.println("Level selected: " + m_autoSelectedLevel);

    // m_autoSelectedReef = m_chooserReef.getSelected();
    // System.out.println("Reef selected: " + m_autoSelectedReef);

  }

  @Override
  public void autonomousPeriodic() {

    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    
  }
}
