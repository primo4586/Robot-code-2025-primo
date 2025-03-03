// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.Commands.positionCommands.updateGlobalPoseWithVision;
import frc.robot.PrimoLib.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Vision _frontCamera = Vision.getFrontCamera();

  public Robot() {
    m_robotContainer = new RobotContainer();
    Elastic.loadPaths();

    Elastic.autoSelector();
    
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
    SmartDashboard.putNumber("angleFromTarget", Vision.getReefCamera().getAngleFromTarget());
    m_robotContainer.log();
    new updateGlobalPoseWithVision(isDisabled()); // update the global pose
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
    
    /*
     * this is to log network tabale. it could log more but you need to tune it. 
     */
    DataLogManager.start();
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
  public void simulationPeriodic() {}
}
