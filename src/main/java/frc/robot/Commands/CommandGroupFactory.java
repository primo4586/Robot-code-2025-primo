package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Misc;
import frc.robot.RobotContainer;
import frc.robot.Commands.swerveCommands.PutCoralTakeAlgea;
import frc.robot.Commands.swerveCommands.driveToPointWithPIDCommand;
import frc.robot.PrimoLib.Elastic;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Gripper.GripperSubsystem;
import frc.robot.subsystems.gripperArm.GripperArm;

import static frc.robot.Misc.*;
import static frc.robot.subsystems.Elevator.ElevatorConstanst.L3_HEIGHT;
import static frc.robot.subsystems.Elevator.ElevatorConstanst.L4_HEIGHT;

import java.util.function.BooleanSupplier;

public class CommandGroupFactory {

    private final static CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private final static GripperSubsystem gripper = GripperSubsystem.getInstance();
    private final static GripperArm gripperArm = GripperArm.getInstance();
    private final static TalonFX motor1 = new TalonFX(1,Misc.CANIVOR_NAME);
    private final static TalonFX motor2 = new TalonFX(4,Misc.CANIVOR_NAME);
    private final static TalonFX motor3 = new TalonFX(7,Misc.CANIVOR_NAME);
    private final static TalonFX motor4 = new TalonFX(10,Misc.CANIVOR_NAME);

    public static Command putCoral(double height){
        return Commands.parallel(elevator.relocatePositionCommand(height) ,Commands.waitUntil(() -> elevator.isAtTarget()).andThen(cannon.loosenCoralCommand())).andThen(elevator.relocatePositionCommand(ElevatorConstanst.L1_HEIGHT));
    }
    public static Command autoCommand(){
        return new InstantCommand(() ->{ motor1.set(0.2);
        motor2.set(0.2);
        motor3.set(0.2);
        motor4.set(0.2);});
    }



}