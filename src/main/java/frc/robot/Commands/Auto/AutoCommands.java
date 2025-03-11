package frc.robot.Commands.Auto;

import java.util.Vector;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Commands.swerveCommands.driveToPointWithPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Vision.Vision;

public class AutoCommands {
    private static DoubleSupplier vector = () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? 1 : -1;
        private final static CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private static Vision _frontCamera = Vision.getFrontCamera();
    private static double MaxSpeed = RobotContainer.MaxSpeed;
                                                                                        // speed
    private static double MaxAngularRate = RobotContainer.MaxAngularRate; // 3/4 of a rotation per second
                                                                                          // max angular velocity
          private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
    
      private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1) // ^joy stick deadband but i'm not shure why we need Max Speed here?
          .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public static Command driveStright(){
        return Commands.runOnce(() -> swerve.setControl(
            drive
                .withVelocityX(0.15 * MaxSpeed * vector.getAsDouble())
                .withVelocityY(0)));
    }

    /**
     * Put coral on level 4 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     * 
     * @return a command that moves the elevator to the correct position and then
     *         loosens the coral.
     */
    public static Command putCoralL4() {
        return Commands.sequence(
                elevator.relocatePositionCommand(ElevatorConstanst.L4_HEIGHT),
                Commands.waitUntil(() -> elevator.isAtTarget()).andThen(cannon.loosenCoralCommand()));
    }

    /**
     * Put coral on level 3 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     */
    public static Command putCoralL3() {
        return Commands.sequence(
                elevator.relocatePositionCommand(ElevatorConstanst.L3_HEIGHT),
                Commands.waitUntil(() -> elevator.isAtTarget()).andThen(cannon.loosenCoralCommand()));
    }

    /**
     * Put coral on level 2 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     */
    public static Command putCoralL2() {
        return Commands.sequence(
                elevator.relocatePositionCommand(ElevatorConstanst.L2_HEIGHT),
                Commands.waitUntil(() -> elevator.isAtTarget()).andThen(cannon.loosenCoralCommand()));
    }

    /**
     * Put coral on level 1 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     */
    public static Command putCoralL1() { // this doesn't work
        return Commands.sequence(
                elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT),
                Commands.waitUntil(() -> elevator.isAtTarget()).andThen(cannon.loosenCoralCommand()));
    }
    
    public static Command normalCommand(){
        return Commands.sequence(driveStright().
        andThen(Commands.waitUntil(() ->_frontCamera.getXfromTarget() < 1 && _frontCamera.getXfromTarget() != 0))
        ,new driveToPointWithPIDCommand(false).withTimeout(2),
        putCoralL4()
        );
    }
}
