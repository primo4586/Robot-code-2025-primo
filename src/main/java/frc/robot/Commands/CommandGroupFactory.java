package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.swerveCommands.driveToPointWithPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Gripper.GripperSubsystem;
import frc.robot.subsystems.gripperArm.GripperArm;

import static frc.robot.Misc.*;
import static frc.robot.subsystems.Elevator.ElevatorConstanst.L4_HEIGHT;

import java.util.function.BooleanSupplier;

public class CommandGroupFactory {

    private final static CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private final static GripperSubsystem gripper = GripperSubsystem.getInstance();
    private final static GripperArm gripperArm = GripperArm.getInstance();

    public static Command putCoralTakeAlgea(double level, double gripperAngle){
        return Commands.parallel(elevator.relocatePositionCommand(ElevatorConstanst.L4_HEIGHT).
        andThen(Commands.waitUntil(() -> elevator.isAtTarget()).andThen(cannon.loosenCoralCommand())));
        // return Commands.sequence(elevator.setTargetPositionCommand(L4_HEIGHT)
        // .andThen(gripperArm.setAngle(0)
        // .andThen(Commands.waitUntil(() -> elevator.isAtTarget()))) 
        // .andThen(cannon.loosenCoralCommand())
        // .andThen(gripperArm.setAngle(gripperAngle).andThen(elevator.setTargetPositionCommand(level)))
        // .andThen(Commands.waitUntil(() -> elevator.isAtTarget()))
        // .andThen(gripper.collectUntilCollectedCommand())
        // ).withName("putCoralTakeAlgea");
    }
}

