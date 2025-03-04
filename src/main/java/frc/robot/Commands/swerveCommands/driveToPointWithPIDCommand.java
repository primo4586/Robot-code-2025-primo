
package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

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

public class driveToPointWithPIDCommand extends Command { // TODO: need to orgeniz and tune evrything + add comments
  private static Pose2d _target;
  private DoubleSupplier vector = () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? 1 : -1;

  static double MaxSpeed = RobotContainer.MaxSpeed;
  static double MaxAngularRate = RobotContainer.MaxSpeed;
  private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;

  private static final SwerveRequest.FieldCentricFacingAngle facingAngel = new FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1) // ^joy stick deadband but i'm not shure why we need Max Speed here?
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /*
   * i dont think we need profile because we use feed foword to control the
   * velocity
   * and we use FOC to control the acceleration.
   * but MA did use profile.
   */
  PIDController driveXPidController = new PIDController(5, 1.5, 0.4); // TODO: check the values again.
  PIDController driveYPidController = new PIDController(5, 1.5, 0.4);

  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithPIDCommand(boolean isRight) {
    addRequirements(swerve);
    _target = PrimoCalc.ChooseReef(isRight);

    driveXPidController.setSetpoint(_target.getX());
    driveXPidController.setTolerance(0.02);
    driveYPidController.setSetpoint(_target.getY());
    driveYPidController.setTolerance(0.02);
  }

  public driveToPointWithPIDCommand(Pose2d target) {
    _target = target;
    driveXPidController.setSetpoint(target.getX());
    driveXPidController.setTolerance(0.02);
    driveYPidController.setSetpoint(target.getY());
    driveYPidController.setTolerance(0.02);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called everey time the scheduler runs while the command is schduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("swerve x error", _target.getX() - swerve.getState().Pose.getX());
    SmartDashboard.putNumber("swerve y error", _target.getY() - swerve.getState().Pose.getY());
    swerve.setControl(
        facingAngel
            .withVelocityX(vector.getAsDouble() * driveXPidController.calculate(swerve.getState().Pose.getX()) * 0.7)
            .withVelocityY(vector.getAsDouble() * driveYPidController.calculate(swerve.getState().Pose.getY()) * 0.7)
            .withTargetDirection(swerve.getState().Pose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.applyRequest(() -> brake); // ^ this might be too much and will kill the robot. the pid it self should stop
                                      // the swerve.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveXPidController.atSetpoint() && driveYPidController.atSetpoint();
  }
}
