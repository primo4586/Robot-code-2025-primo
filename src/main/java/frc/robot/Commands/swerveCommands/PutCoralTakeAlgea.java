package frc.robot.Commands.swerveCommands;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Gripper.GripperSubsystem;
import frc.robot.subsystems.gripperArm.GripperArm;

public class PutCoralTakeAlgea extends SequentialCommandGroup {

    public  PutCoralTakeAlgea(CannonSubsystem cannon, GripperArm gripperArm, ElevatorSubsystem elevator, 
    GripperSubsystem gripper,double level, double gripperAngle){
        SequentialCommandGroup command =  new SequentialCommandGroup(
        elevator.setTargetPositionCommand(ElevatorConstanst.L4_HEIGHT),
        elevator.relocatePositionCommand(),
        gripperArm.setAngle(0)
        ,Commands.waitUntil(() -> elevator.isAtTarget()) 
        ,cannon.loosenCoralCommand()
        ,gripperArm.setAngle(gripperAngle),
        elevator.setTargetPositionCommand(level),
        elevator.relocatePositionCommand()
        ,Commands.waitUntil(() -> elevator.isAtTarget())
        ,gripper.collectUntilCollectedCommand()
        .withName("putCoralTakeAlgea"));

        addCommands(command);

    }

    
    
}
