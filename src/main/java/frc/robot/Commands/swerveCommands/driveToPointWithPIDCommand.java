

package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.PrimoLib.PrimoCalc;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class driveToPointWithPIDCommand extends Command {
  private static Pose2d target;

  static double  MaxSpeed = RobotContainer.MaxSpeed;
  static double  MaxAngularRate = RobotContainer.MaxSpeed;
  private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
  private static final SwerveRequest.FieldCentric roborCentric = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  ProfiledPIDController driveXPidController = new ProfiledPIDController(0.355,0,0.01,new Constraints(0, 0));
  ProfiledPIDController driveYPidController = new ProfiledPIDController(0.355,0,0.01,new Constraints(0, 0));
  PIDController rotionPidController = new PIDController(0.05, 0, 0);

  CommandXboxController joyStick = RobotContainer._driverController;

  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithPIDCommand(boolean isRight) {
    target = PrimoCalc.ChooseReef(isRight);
    rotionPidController.enableContinuousInput(180, -180);
    rotionPidController.setSetpoint(target.getRotation().getRadians());
    rotionPidController.setTolerance(1);

    driveXPidController.setGoal(target.getX());
    driveXPidController.setTolerance(0.02);
    driveYPidController.setGoal(target.getY());
    driveYPidController.setTolerance(0.02);
  }

  public driveToPointWithPIDCommand(Pose2d target){
    this.target = target;
    rotionPidController.enableContinuousInput(180, -180);
    rotionPidController.setSetpoint(target.getRotation().getDegrees());
    rotionPidController.setTolerance(1);

    driveXPidController.setGoal(target.getX());
    driveXPidController.setTolerance(0.02);
    driveYPidController.setGoal(target.getY());
    driveYPidController.setTolerance(0.02);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called everey time the scheduler runs while the command is schduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("swerve x error", target.getX() - swerve.getState().Pose.getX());
    SmartDashboard.putNumber("swerve y error", target.getY() - swerve.getState().Pose.getY());
    swerve.setControl(
      roborCentric.withVelocityX(driveXPidController.calculate(swerve.getState().Pose.getX()))
        .withVelocityY(driveYPidController.calculate(swerve.getState().Pose.getY()))
        .withRotationalRate(joyStick.getRightX() * MaxAngularRate * 0.7)
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
    return driveXPidController.atGoal() && driveYPidController.atGoal();
  }
}
