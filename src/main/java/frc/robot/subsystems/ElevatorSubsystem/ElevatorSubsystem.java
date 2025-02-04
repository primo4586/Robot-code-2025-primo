// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.ElevatorSubsystem.ElevatorConstanst.*;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_masterMotor; // falcon 500
  private TalonFX m_followMotor; // falcon 500
  private Follower follower;

  private static MotionMagicVoltage _systemControl = new MotionMagicVoltage(0);

  private static DoubleSupplier targetPosition = () -> 0.0;

  private static ElevatorSubsystem instance;
  /**
   * Gets the instance of ElevatorSubsystem. This is the only way to get an
   * instance of the class, as the constructor is private.
   *
   * @return the instance of ElevatorSubsystem
   */
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }
  private ElevatorSubsystem() {
    m_masterMotor = new TalonFX(MASTER_TALONFX_ID);
    m_followMotor = new TalonFX(FOLLOW_TALONFX_ID);
    follower = new Follower(MASTER_TALONFX_ID, true);
    resetElevator();
    configs();
  }

  /**
   * Reset the elevator to its default position at the ground.
   *
   * <p>This is useful for making sure the elevator is in a safe position
   * and for knowing where the elevator is when the program starts.
   */
  public void resetElevator(){
    m_masterMotor.setPosition(HIGHT_OF_THE_GROUND);
  }
  /**
   * Sets the target position for the elevator.
   *
   * @param position the desired target position in metters.
   */
  public void setTargetPosition(double position) {
    targetPosition = () -> position;
  }
  /**
   * A command that moves the elevator to its target position.
   * You can set the target position using the {@link #setTargetPosition(double)} method.
   * and you should call this Command once per match
   */
  public Command relocatePositionCommand() {
    return run(() -> 
    {
      m_masterMotor.setControl(_systemControl.withPosition(targetPosition.getAsDouble()).withEnableFOC(true));
      m_followMotor.setControl(follower);
    });
  }
  /**
   * a Command that moves the Elavator at a constant power {@link #MOVE_POWER}
   * with direction of the parameter and stop the elevator
   * when butten is relesd, then set the motor to overcome gravity and stay in place. 
   * @param vec a value that is 1 or -1 depending on the direction
   * @return
   */
  public Command moveCommand(int vec){
    return runEnd(() -> 
    {
      m_masterMotor.set(MOVE_POWER * vec);
      m_followMotor.setControl(follower);
    }, 
    () -> 
      m_masterMotor.set(0.3) // kg
    );
  }

  /**
   * Check if the elevator is at its target position.
   *
   * <p>This method returns true if the elevator is close enough to its target
   * position and not moving quickly enough to be considered at its target
   * position. The threshold for "close enough" is defined in {@link #MINIMUN_POSITION_ERROR}
   * and the threshold for "not moving quickly enough" is defined in
   * {@link #MINIMUN_VELOCITY_ERROR}.
   *
   * @return true if the elevator is at its target position, false otherwise.
   */
  public boolean isAtTarget() {
    return Math.abs(m_masterMotor.getPosition().getValueAsDouble() - targetPosition.getAsDouble()) < MINIMUN_POSITION_ERROR
            && Math.abs(m_masterMotor.getVelocity().getValueAsDouble()) < MINIMUN_VELOCITY_ERROR;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elavator/IsAtTarget", isAtTarget());
    SmartDashboard.putNumber("Elavator/measurePosition", m_masterMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elavator/wantedPosition", targetPosition.getAsDouble());
  }

  private void configs(){
    m_masterMotor.getConfigurator().apply(ELEVATOR_CONFIG);
    m_followMotor.getConfigurator().apply(ELEVATOR_CONFIG);
  }
}
