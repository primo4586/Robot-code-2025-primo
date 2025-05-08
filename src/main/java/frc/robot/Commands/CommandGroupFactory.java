package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import javax.xml.crypto.dsig.CanonicalizationMethod;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.swerveCommands.DriveToDistanceWithCamera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Disposer.Disposer;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class CommandGroupFactory {
    private static final Disposer disposer = Disposer.getInstance();
    private static final CannonSubsystem cannon = CannonSubsystem.getInstance();
    private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
    private static double MaxSpeed = RobotContainer.MaxSpeed;
      private static final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1) // ^joy stick deadband but i'm not shure why we need Max Speed here?
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static int rightLevel = 4; // wich level is free in each side
    private static int leftLevel = 4;
    private static DoubleSupplier wantedHight = () -> 4 == rightLevel ? ElevatorConstanst.L4_HEIGHT :
    3 == rightLevel ? ElevatorConstanst.L3_HEIGHT :
    ElevatorConstanst.L2_HEIGHT;

    public static Command drive(int vector){
        return Commands.startEnd(() -> swerve.setControl(
            drive
                .withVelocityX(0.2 * MaxSpeed * vector)
                .withVelocityY(0))
                ,() -> swerve.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0))).withTimeout(0.6);
    }

    public static Command getAlgeaOut(){
        return Commands.sequence(disposer.preparingCommand(),new DriveToDistanceWithCamera(true),disposer.goHomeCommand(),drive(-1));
    }

    public static Command removeAlgeaFromL2(){
        return Commands.sequence(elevator.relocatePositionCommand(ElevatorConstanst.L2_HEIGHT), Commands.waitSeconds(1),
        getAlgeaOut(), elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT));
    }

    public static Command safePlaceCoral(boolean isRight){ // 90% this wouldn't work, there is no wait!
        return Commands.sequence(elevator.relocatePositionCommand(ElevatorConstanst.L2_HEIGHT),
        new DriveToDistanceWithCamera(isRight),
        cannon.loosenCoralCommand(),
        elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT)
        );
    }

    public static Command safePlaceRight(){
        return Commands.sequence( 
            elevator.relocatePositionCommand(ElevatorConstanst.L3_HEIGHT),
            new DriveToDistanceWithCamera(true),
            elevator.relocatePositionCommand(wantedHight),
            Commands.waitUntil(() -> elevator.isAtTarget()),
            cannon.loosenCoralCommand(),
            elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT),
            Commands.runOnce(() -> {rightLevel --; System.out.println(wantedHight.getAsDouble() + " " + rightLevel);}),
            new ConditionalCommand(Commands.runOnce(() -> rightLevel = 4), Commands.none(), () -> rightLevel == 1)
            );
    }
}

