// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.PrimoLib.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private String m_autoSelectedReef;
  
  private String m_autoSelectedLevel;
  private final RobotContainer m_robotContainer;
  private final Vision _frontCamera = Vision.getFrontCamera();
  private final Vision _leftCamera = Vision.getLeftCamera();
  private final Vision _rightCamera = Vision.getRightCamera();

  private final boolean kUseLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Elastic.autoSelector();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ElevatorSubsystem.getInstance().setDefaultCommand(ElevatorSubsystem.getInstance().relocatePositionCommand());
    Elastic.dispalyCommandScheduler();
    m_robotContainer.log();

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

                m_robotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });

    // right camera
    visionEst = _rightCamera.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = _frontCamera.getEstimationStdDevs();

                m_robotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });

    // left camera
    visionEst = _leftCamera.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = _frontCamera.getEstimationStdDevs();

                m_robotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
