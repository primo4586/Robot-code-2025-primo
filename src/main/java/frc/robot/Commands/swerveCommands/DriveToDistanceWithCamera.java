// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.PrimoLib.PrimoCalc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;

import static frc.robot.Commands.swerveCommands.SwerveCommandsConstants.*;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToDistanceWithCamera extends Command {
  private DoubleSupplier vector = () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? 1 : -1;
  private boolean isRight;
  private static Pose2d _target;
  private Vision _Camera;
  private static Pose2d _CameraTarget;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  PIDController driveXPidController;
  PIDController driveYPidController;
  PIDController driveRotationPidController;

  private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;

  private static final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private static final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private static double velocityX;
  private static double velocityY;
  private static double angularRate;

  /*
   * i dont think we need profile because we use feed foword to control the
   * velocity
   * and we use FOC to control the acceleration.
   * but MA did use profile.
   */

  public DriveToDistanceWithCamera(boolean isRight) {
    addRequirements(swerve);
    this.isRight = isRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Camera = this.isRight ? Vision.getLeftCamera() : Vision.getRightCamera();
    _CameraTarget = this.isRight ? VisionConstants.rightReefTargetGoal : VisionConstants.leftReefTargetGoal;
    driveXPidController = new PIDController(3.7, 0, 0);
    driveYPidController = new PIDController(4, 0, 0);
    driveRotationPidController = new PIDController(2.1, 0, 0);
    driveRotationPidController.enableContinuousInput(-Math.PI, Math.PI);
    driveXPidController.setTolerance(0.01);
    driveYPidController.setTolerance(0.01);
    driveXPidController.setSetpoint(0);
    driveYPidController.setSetpoint(0);
    _target = PrimoCalc.ChooseReef(this.isRight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ;
    if (_Camera.getDetectingObject()) {
      velocityX = -driveXPidController.calculate(_Camera.getXfromTarget(), _CameraTarget.getX());
      velocityY = -driveYPidController.calculate(_Camera.getYfromTarget(), _CameraTarget.getY());
      angularRate = MAX_ANGULAR_RATE * driveRotationPidController.calculate(_Camera.getAngleFromTarget(),
          _CameraTarget.getRotation().getRadians());
      if (Math.abs(velocityX) > MAX_VELOCITY_X)
        velocityX = MAX_VELOCITY_X;

      if (Math.abs(velocityY) > MAX_VELOCITY_Y)
        velocityY = MAX_VELOCITY_Y;

      if (Math.abs(angularRate) > MAX_ANGULAR_RATE)
        angularRate = MAX_ANGULAR_RATE;

      swerve.setControl(
          robotCentric
              .withVelocityX(velocityX)
              .withVelocityY(velocityY)
              .withRotationalRate(angularRate));
    } else {
      swerve.setControl(
          fieldCentric
              .withVelocityX(
                  vector.getAsDouble() * driveXPidController.calculate(swerve.getState().Pose.getX(), _target.getX()))
              .withVelocityY(
                  vector.getAsDouble() * driveYPidController.calculate(swerve.getState().Pose.getY(), _target.getY())));
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(
        robotCentric
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    System.out.println(
        _Camera.getTargetID()
            + " " +
            isRight
            + " X: " +
            swerve.getState().Pose.getX()
            + " Y: " +
            swerve.getState().Pose.getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveXPidController.atSetpoint() && driveYPidController.atSetpoint();
  }
}
