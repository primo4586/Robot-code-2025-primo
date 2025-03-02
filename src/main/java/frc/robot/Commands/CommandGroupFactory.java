package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.swerveCommands.driveToPointWithPIDCommand;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class CommandGroupFactory {

    private final static CannonSubsystem cannon = CannonSubsystem.getInstance();
    private final static ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    public static Command putCoral(double level){
        return Commands.parallel(elevator.relocatePositionCommand(level).
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

    public static Command autoCommand(){
        return Commands.parallel(new driveToPointWithPIDCommand(false).withTimeout(5)
        .andThen(putCoral(ElevatorConstanst.L4_HEIGHT)));
    }


}

