package frc.robot.Commands.swerveCommands;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Gripper.GripperSubsystem;
import frc.robot.subsystems.gripperArm.GripperArm;

public class PutCoralTakeAlgea extends SequentialCommandGroup {

    private final CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private final GripperSubsystem gripper = GripperSubsystem.getInstance();
    private final GripperArm gripperArm = GripperArm.getInstance();
    public  PutCoralTakeAlgea(double level, double gripperAngle){



        ParallelDeadlineGroup preparePutCoral = new ParallelDeadlineGroup(
            Commands.waitSeconds(0.02).andThen(Commands.waitUntil(() -> elevator.isAtTarget())),
            elevator.relocatePositionCommand(ElevatorConstanst.L4_HEIGHT),
            gripperArm.relocateAngelCommand(0));


            ParallelDeadlineGroup prepateTakeAlgea = new ParallelDeadlineGroup(
                Commands.waitSeconds(0.02).andThen(
                Commands.waitUntil(() -> elevator.isAtTarget())),
                elevator.relocatePositionCommand(level),
                gripperArm.relocateAngelCommand(gripperAngle));


        // SequentialCommandGroup command =  new SequentialCommandGroup(
        // elevator.setTargetPositionCommand(ElevatorConstanst.L4_HEIGHT),
        // elevator.relocatePositionCommand(),
        // gripperArm.setAngle(0)
        // ,Commands.waitUntil(() -> elevator.isAtTarget()) 
        // ,cannon.loosenCoralCommand()
        // ,gripperArm.setAngle(gripperAngle),
        // elevator.setTargetPositionCommand(level),
        // elevator.relocatePositionCommand()
        // ,Commands.waitUntil(() -> elevator.isAtTarget())
        // ,gripper.collectUntilCollectedCommand()
        // .withName("putCoralTakeAlgea"));

        addCommands(preparePutCoral,
                    cannon.loosenCoralCommand(),
                    prepateTakeAlgea,
                    gripper.collectUntilCollectedCommand()
                    );

    }

    
    
}
