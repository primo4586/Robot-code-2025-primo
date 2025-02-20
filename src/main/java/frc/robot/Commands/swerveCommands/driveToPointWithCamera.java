package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.PrimoLib.PrimoCalc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;


public class driveToPointWithCamera extends Command {
  static double  MaxSpeed = RobotContainer.MaxSpeed;
      static double  MaxAngularRate = RobotContainer.MaxSpeed;
            private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
            private static final SwerveRequest.RobotCentric roborCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  PIDController driveXPidController = new PIDController(0.285,0,0.01);
  PIDController driveYPidController = new PIDController(0.285,0,0.01);
  PIDController rotionPidController = new PIDController(0.05, 0, 0);

  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithCamera() {
    rotionPidController.enableContinuousInput(180, -180);
    rotionPidController.setSetpoint(0);
    rotionPidController.setTolerance(1);

    driveXPidController.setSetpoint(0);
    driveXPidController.setTolerance(0.03);
    driveYPidController.setSetpoint(-0.02);
    driveYPidController.setTolerance(0.03);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called everey time the scheduler runs while the command is schduled.
  @Override
  public void execute() {
    swerve.setControl(
      roborCentric.withVelocityX(driveXPidController.calculate(Vision.getFrontCamera().getXfromTarget()) * RobotContainer.MaxSpeed)
        .withVelocityY(driveYPidController.calculate(Vision.getFrontCamera().getYfromTarget() -0.02) * RobotContainer.MaxSpeed)
        
    );
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.applyRequest(() -> brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveXPidController.atSetpoint() && driveYPidController.atSetpoint();
  }
}

