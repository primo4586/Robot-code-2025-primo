package frc.robot.Commands.swerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static frc.robot.subsystems.Vision.VisionConstants.leftReefTargetGoal;
import static frc.robot.subsystems.Vision.VisionConstants.rightReefTargetGoal;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.PrimoLib.PrimoCalc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;

public class driveToPointWithCamera extends Command {
  static double MaxSpeed = RobotContainer.MaxSpeed;
  static double MaxAngularRate = RobotContainer.MaxSpeed;
  private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
  private static final SwerveRequest.FieldCentric roborCentric = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Vision _reefCamera = Vision.getReefCamera();

  private Pose2d globalGoal; // with global pose astimation (red camera)
  private Pose2d goal; // between cameraGoal and globalGoal
  private Pose2d cameraGoal; // reef's camera pos

  private Pose2d currentPoseGlobal;
  private Pose2d currentPoseCamera;
  private Pose2d currentPose;
 




  PIDController driveXPidController = new PIDController(3.5, 0, 0.0);
  PIDController driveYPidController = new PIDController(3.5, 0, 0.0);
  PIDController rotionPidController = new PIDController(0.05, 0, 0);

  Timer timer = new Timer();

  /** Creates a new driveToPointWithPIDCommand. */
  public driveToPointWithCamera(boolean isRight) {
    globalGoal = PrimoCalc.ChooseReef(isRight);
    cameraGoal = isRight ? rightReefTargetGoal : leftReefTargetGoal;


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

    goal = _reefCamera.getDetectingObject() ?
     new Pose2d(cameraGoal.getX(), cameraGoal.getY(), cameraGoal.getRotation()) 
    : new Pose2d(globalGoal.getX(), globalGoal.getY(), globalGoal.getRotation());

    driveXPidController.setSetpoint(goal.getX());
    driveYPidController.setSetpoint(goal.getY());
    rotionPidController.setSetpoint(goal.getRotation().getRadians());

    currentPoseGlobal = RobotContainer.drivetrain.getState().Pose;
    currentPoseCamera = new Pose2d
    (_reefCamera.getXfromTarget(),
    _reefCamera.getYfromTarget(),
    new Rotation2d (_reefCamera.getAngleFromTarget()));

    currentPose = _reefCamera.getDetectingObject() ?
    new Pose2d(currentPoseCamera.getX(), currentPoseCamera.getY(), currentPoseCamera.getRotation()) 
   : new Pose2d(currentPoseGlobal.getX(), currentPoseGlobal.getY(), currentPoseGlobal.getRotation());




    swerve.setControl(
        roborCentric
            .withVelocityX(
                -driveXPidController.calculate(currentPose.getX()) * 0.01)
            .withVelocityY(-driveYPidController.calculate(currentPose.getY()) * 0.01)
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
