package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;

public class driveToPointWithCamera extends Command {
  static double MaxSpeed = RobotContainer.MaxSpeed;
  static double MaxAngularRate = RobotContainer.MaxSpeed;
  private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
  private static final SwerveRequest.RobotCentric roborCentric = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  PIDController driveXPidController = new PIDController(3.5, 0, 0.0);
  PIDController driveYPidController = new PIDController(3.5, 0, 0.0);
  PIDController rotionPidController = new PIDController(0.05, 0, 0);

  Timer timer = new Timer();

  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithCamera() {
    rotionPidController.enableContinuousInput(180, -180);
    rotionPidController.setSetpoint(0);
    rotionPidController.setTolerance(1);

    driveXPidController.setSetpoint(0.9);
    driveXPidController.setTolerance(0.03);
    driveYPidController.setSetpoint(0.1);
    driveYPidController.setTolerance(0.03);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called everey time the scheduler runs while the command is schduled.
  @Override
  public void execute() {
    swerve.setControl(
        roborCentric
            .withVelocityX(
                -driveXPidController.calculate(Vision.getFrontCamera().getXfromTarget()))
            .withVelocityY(-driveYPidController.calculate(Vision.getFrontCamera().getYfromTarget()))
    );

    if(!driveXPidController.atSetpoint() || !driveYPidController.atSetpoint()){
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.applyRequest(() -> brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.5);
  }
}
