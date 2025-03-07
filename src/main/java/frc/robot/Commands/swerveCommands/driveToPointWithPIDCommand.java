
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
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withHeadingPID(7, 0, 0);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /*
   * i dont think we need profile because we use feed foword to control the
   * velocity
   * and we use FOC to control the acceleration.
   * but MA did use profile.
   */
  PIDController driveXPidController = new PIDController(4, 1.5, 0); // TODO: check the values again.
  PIDController driveYPidController = new PIDController(4, 1.5, 0);

  private boolean isRight = false;

  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithPIDCommand(boolean isRight) {
    addRequirements(swerve);
    this.isRight = isRight;
    
    driveXPidController.setTolerance(0.02);
    driveYPidController.setTolerance(0.02);
  }

  public driveToPointWithPIDCommand(Pose2d target) {
    addRequirements(swerve);
    _target = target;
    driveXPidController.setTolerance(0.02);
    driveYPidController.setTolerance(0.02);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called everey time the scheduler runs while the command is schduled.
  @Override
  public void execute() {
    _target = PrimoCalc.ChooseReef(this.isRight);
    driveXPidController.setSetpoint(_target.getX());
    driveYPidController.setSetpoint(_target.getY());
    SmartDashboard.putNumber("swerve x error", driveXPidController.getError());
    SmartDashboard.putNumber("swerve y error", driveYPidController.getError());
    // SmartDashboard.putNumber("swerve rot error", swerve.getState().Pose.getRotation());
    swerve.setControl(
        facingAngel
            .withVelocityX(vector.getAsDouble() * driveXPidController.calculate(swerve.getState().Pose.getX()))
            .withVelocityY(vector.getAsDouble() * driveYPidController.calculate(swerve.getState().Pose.getY()))
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
