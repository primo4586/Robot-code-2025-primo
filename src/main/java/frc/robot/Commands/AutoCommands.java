package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Cannon.CannonSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstanst;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class AutoCommands {
    private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private static final CannonSubsystem cannon = CannonSubsystem.getInstance();

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
                elevator.setTargetPositionCommand(ElevatorConstanst.L4_HEIGHT),
                Commands.waitUntil(elevator::isAtTarget).andThen(cannon.loosenCoralCommand()));
    }

    /**
     * Put coral on level 3 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     */
    public static Command putCoralL3() {
        return Commands.sequence(
                elevator.setTargetPositionCommand(ElevatorConstanst.L3_HEIGHT),
                Commands.waitUntil(elevator::isAtTarget).andThen(cannon.loosenCoralCommand()));
    }

    /**
     * Put coral on level 2 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     */
    public static Command putCoralL2() {
        return Commands.sequence(
                elevator.setTargetPositionCommand(ElevatorConstanst.L2_HEIGHT),
                Commands.waitUntil(elevator::isAtTarget).andThen(cannon.loosenCoralCommand()));
    }

    /**
     * Put coral on level 1 of the reef.
     * <p>
     * This command moves the elevator to the correct position and then loosens the
     * coral.
     */
    public static Command putCoralL1() { // this doesn't work
        return Commands.sequence(
                elevator.setTargetPositionCommand(ElevatorConstanst.L1_HEIGHT),
                Commands.waitUntil(elevator::isAtTarget).andThen(cannon.loosenCoralCommand()));
    }
}
