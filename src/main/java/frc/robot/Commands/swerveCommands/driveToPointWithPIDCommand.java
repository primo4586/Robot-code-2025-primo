

package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.PrimoLib.PrimoCalc;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class driveToPointWithPIDCommand extends Command { //todo: need to orgeniz and tune evrything 
  private static Pose2d _target;
                private DoubleSupplier vector = () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? 1 : -1;

  static double  MaxSpeed = RobotContainer.MaxSpeed;
  static double  MaxAngularRate = RobotContainer.MaxSpeed;
  private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
  private static final SwerveRequest.FieldCentric roborCentric = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  PIDController driveXPidController = new PIDController(5, 1.5, 0.4);
  PIDController driveYPidController = new PIDController(5, 1.5, 0.4);
  PIDController rotionPidController = new PIDController(0.025, 0, 0.01);
  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithPIDCommand(boolean isRight) {
    addRequirements(swerve);
    _target = PrimoCalc.ChooseReef(isRight);
    rotionPidController.enableContinuousInput(180, -180);
    rotionPidController.setSetpoint(_target.getRotation().getRadians());
    rotionPidController.setTolerance(1);

    driveXPidController.setSetpoint(_target.getX());
    driveXPidController.setTolerance(0.02);
    driveYPidController.setSetpoint(_target.getY());
    driveYPidController.setTolerance(0.02);
  }

  public driveToPointWithPIDCommand(Pose2d target){
    _target = target;
    rotionPidController.enableContinuousInput(180, -180);
    rotionPidController.setSetpoint(target.getRotation().getDegrees());
    rotionPidController.setTolerance(1);

    driveXPidController.setSetpoint(target.getX());
    driveXPidController.setTolerance(0.02);
    driveYPidController.setSetpoint(target.getY());
    driveYPidController.setTolerance(0.02);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called everey time the scheduler runs while the command is schduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("swerve x error", _target.getX() - swerve.getState().Pose.getX());
    SmartDashboard.putNumber("swerve y error", _target.getY() - swerve.getState().Pose.getY());
    swerve.setControl(
      roborCentric.withVelocityX(  vector.getAsDouble() * driveXPidController.calculate(swerve.getState().Pose.getX()) * 0.7)
        .withVelocityY(vector.getAsDouble() * driveYPidController.calculate(swerve.getState().Pose.getY()) * 0.7)
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
