// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.PrimoLib.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private String m_autoSelectedReef;
  private int robotRotation;
  private String m_autoSelectedLevel;
  private final RobotContainer m_robotContainer;
  private final Vision _frontCamera = Vision.getFrontCamera();
  
  private double[] cameraPoseArray = new double[] {0, 0, 0};

  private final boolean kUseLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Elastic.loadPaths();

    Elastic.autoSelector();
    
    Elastic.displayField();
    m_robotContainer.drivetrain.setStateStdDevs(VisionConstants.kSingleTagStdDevs);
    
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
    // SmartDashboard.putNumber("getXfromTarget", Vision.getFrontCamera().getXfromTarget());
    // SmartDashboard.putNumber("getYfromTarget", Vision.getFrontCamera().getYfromTarget());

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */

    // Correct pose estimate with vision measurements
    // front camera 
    // !this has not been tested
    var visionEst = _frontCamera.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = _frontCamera.getEstimationStdDevs();

                RobotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
            });
          }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

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
