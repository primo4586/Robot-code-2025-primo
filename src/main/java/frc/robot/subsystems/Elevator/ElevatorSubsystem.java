// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Misc;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Elevator.ElevatorConstanst.*;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_masterMotor; // falcon 500
  private TalonFX m_followMotor; // falcon 500
  private Follower follower;
  //sysid
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null ,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_masterMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );


  private static MotionMagicVoltage _systemControl = new MotionMagicVoltage(0);

  private static double targetPosition = 0.0;

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
    m_masterMotor = new TalonFX(MASTER_TALONFX_ID, Misc.CANIVOR_NAME);
    m_followMotor = new TalonFX(FOLLOW_TALONFX_ID, Misc.CANIVOR_NAME);
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
   * Reset the elevator to its default position at the ground (as a command);
   *
   * <p>This is useful for making sure the elevator is in a safe position
   * and for knowing where the elevator is when the program starts.
   */
  public Command resetElevatorCommand(){
    return runOnce(() -> resetElevator()).withName("reset elevator");
  }

  /**
   * Sets the target position for the elevator.
   *
   * @param position the desired target position in metters.
   */
  public void setTargetPosition(double position) {
    targetPosition = position;
  }

   /**
   * Sets the target position for the elevator. (as a command)
   *
   * @param position the desired target position in metters.
   */
  public Command setTargetPositionCommand(double position) {
    return runOnce(() -> setTargetPosition(position)).withName("set elevator target to " + position);
  }
  /**
   * A command that moves the elevator to its target position.
   * You can set the target position using the {@link #setTargetPosition(double)} method.
   * and you should call this Command once per match
   */
  public Command relocatePositionCommand() {
    return run(() -> 
    {
      m_masterMotor.setControl(_systemControl.withPosition(targetPosition));
      m_followMotor.setControl(follower); // following the master
    }).withName("Relocate elevator to " + targetPosition);
  }

  public Command relocatePositionCommand(double angel) {

    return run(() -> 
    {
      targetPosition = angel;
      m_masterMotor.setControl(_systemControl.withPosition(targetPosition));
      m_followMotor.setControl(follower); // following the master
    }).withName("Relocate elevator to " + angel);
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
      targetPosition = m_masterMotor.getPosition().getValueAsDouble() // todo: fixed this part
      ).withName("move elevator Command" + MOVE_POWER * vec);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
 }
 
 public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
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
    return Math.abs(m_masterMotor.getPosition().getValueAsDouble() - targetPosition) < MINIMUN_POSITION_ERROR
            && Math.abs(m_masterMotor.getVelocity().getValueAsDouble()) < MINIMUN_VELOCITY_ERROR;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elavator/IsAtTarget", isAtTarget());
    SmartDashboard.putNumber("Elavator/measurePosition", m_masterMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elavator/wantedPosition", targetPosition);
    SignalLogger.writeDouble("voltage", m_masterMotor.getMotorVoltage().getValueAsDouble());
    SignalLogger.writeDouble("Velocity", m_masterMotor.getVelocity().getValueAsDouble());
    SignalLogger.writeDouble("position", m_masterMotor.getPosition().getValueAsDouble());
  }

  private void configs(){

        //upload configs motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      statusCode = m_masterMotor.getConfigurator().apply(ELEVATOR_CONFIG);
      if(statusCode.isOK())
      break;
    }
    if (!statusCode.isOK())
    System.out.println("Elevator master motor couldn't apply configs" + statusCode.toString());

    statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      statusCode = m_followMotor.getConfigurator().apply(ELEVATOR_CONFIG);
      if(statusCode.isOK())
      break;
    }
    if (!statusCode.isOK())
    System.out.println("Elevator follow motor couldn't apply configs" + statusCode.toString());
  }
}
