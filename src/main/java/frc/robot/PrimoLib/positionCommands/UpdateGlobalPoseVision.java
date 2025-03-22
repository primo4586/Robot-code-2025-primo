package frc.robot.PrimoLib.positionCommands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.Vision;

public class UpdateGlobalPoseVision {
    private static Vision _leftCamera = Vision.getLeftCamera();
    private static Vision _rightCamera = Vision.getRightCamera();
    private static double[] frontCameraEstimatePosition = {0.0,0.0,0.0};

    public static void updateGlobalPoseVision(boolean disabled){
            var visionEst = _leftCamera.getEstimatedGlobalPose(); //the vision estimate
            var visionEstRight = _rightCamera.getEstimatedGlobalPose();
    visionEstRight.ifPresent(
        est -> { 
          if (true) { // if the robot is disabled then we trust the vision rotation
            var estStdDevs = _rightCamera.getEstimationStdDevs(); //the estimation standard deviations
                RobotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds(), estStdDevs);
          }
        }
    );
    visionEst.ifPresent(
        est -> { //if there is a vision estimate

          if (true) { // if the robot is disabled then we trust the vision rotation
            var estStdDevs = _leftCamera.getEstimationStdDevs(); //the estimation standard deviations
                RobotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds(), estStdDevs);
          }
          //  else { // if the robot is not disabled then we dont trust the vision rotation.
          //   var estStdDevs = _leftCamera.getEstimationStdDevs(); //the estimation standard deviations
          //   RobotContainer.drivetrain.addVisionMeasurement( // add the vision measurement to the global pose estimator
          //       new Pose2d(est.estimatedPose.toPose2d().getTranslation(), // add the vision Translation to the global pose estimator
          //           RobotContainer.drivetrain.getState().Pose.getRotation()), // add the odometry rotation
          //       Utils.fpgaToCurrentTime(est.timestampSeconds), // add the timestamp
          //       estStdDevs); // add the estimation standard deviations
          // }
        
        // display the vision estimate
        frontCameraEstimatePosition[0] = visionEst.get().estimatedPose.toPose2d().getTranslation().getX();
        frontCameraEstimatePosition[1] = visionEst.get().estimatedPose.toPose2d().getTranslation().getY();
        frontCameraEstimatePosition[2] = visionEst.get().estimatedPose.toPose2d().getRotation().getDegrees();
        SmartDashboard.putNumberArray("frontCameraEstimatePosition", frontCameraEstimatePosition);
        });
        
        
  }
    }
