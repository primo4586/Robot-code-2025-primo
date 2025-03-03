// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.positionCommands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.Vision;

public class updateGlobalPoseWithVision extends InstantCommand {
  private static boolean _disabled;
  private static final Vision _frontCamera = Vision.getFrontCamera();
  private static double[] frontCameraEstimatePosition = new double[3];

  /**
   * this command updates the global pose estimator with the vision estimate
   * @param disabled - true if the robot is disabled
   */
  public updateGlobalPoseWithVision(boolean disabled) {
    _disabled = disabled;

  }

  @Override
  public void initialize() {
    var visionEst = _frontCamera.getEstimatedGlobalPose(); //the vision estimate

    visionEst.ifPresent(
        est -> { //if there is a vision estimate

          if (_disabled) { // if the robot is disabled then we trust the vision rotation
            var estStdDevs = _frontCamera.getEstimationStdDevs(); //the estimation standard deviations
            RobotContainer.drivetrain.addVisionMeasurement( // add the vision measurement to the global pose estimator
                est.estimatedPose.toPose2d(), // add the vision measurement to the global pose estimator with the vision rotation
                Utils.fpgaToCurrentTime(est.timestampSeconds), // add the timestamp
                estStdDevs); // add the estimation standard deviations

          } else { // if the robot is not disabled then we dont trust the vision rotation.
            var estStdDevs = _frontCamera.getEstimationStdDevs(); //the estimation standard deviations
            RobotContainer.drivetrain.addVisionMeasurement( // add the vision measurement to the global pose estimator
                new Pose2d(est.estimatedPose.toPose2d().getTranslation(), // add the vision Translation to the global pose estimator
                    RobotContainer.drivetrain.getState().Pose.getRotation()), // add the odometry rotation
                Utils.fpgaToCurrentTime(est.timestampSeconds), // add the timestamp
                estStdDevs); // add the estimation standard deviations
          }
        });

        //display the vision estimate
        frontCameraEstimatePosition[0] = visionEst.get().estimatedPose.toPose2d().getTranslation().getX();
        frontCameraEstimatePosition[1] = visionEst.get().estimatedPose.toPose2d().getTranslation().getY();
        frontCameraEstimatePosition[2] = visionEst.get().estimatedPose.toPose2d().getRotation().getDegrees();
        SmartDashboard.putNumberArray("frontCameraEstimatePosition", frontCameraEstimatePosition);
  }

}
