package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Commands.swerveCommands.DriveToDistanceWithCamera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Disposer.Disposer;

public class CommandGroupFactory {
    private static final Disposer disposer = Disposer.getInstance();
    private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
    private static double MaxSpeed = RobotContainer.MaxSpeed;
      private static final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1) // ^joy stick deadband but i'm not shure why we need Max Speed here?
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
        return Commands.sequence(drive(-1),disposer.preparingCommand(),new DriveToDistanceWithCamera(true),disposer.goHomeCommand(),drive(-1));
    }
}

