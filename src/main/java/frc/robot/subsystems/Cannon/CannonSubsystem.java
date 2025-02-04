package frc.robot.subsystems.Cannon;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Misc.*;
import static frc.robot.subsystems.Cannon.CannonConstants.*;


public class CannonSubsystem extends SubsystemBase {

  private SparkMax m_motor;
  private DigitalInput m_sensor;

  //singlton
  private static CannonSubsystem instance;


  public static CannonSubsystem getInstance(){
    if (instance == null){
      instance = new CannonSubsystem();
    }
    return instance;
  }


  private CannonSubsystem(){
    m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    m_sensor = new DigitalInput(SENSOR_ID);
    configs();
  }


  /**
   * collects until the sensor is true
   */
  public Command collectUntilCoralCommand(){
    return startEnd(() -> m_motor.set(MOTOR_SPEED),() -> m_motor.stopMotor()).until(() -> m_sensor.get());
  }


  /**
   * collects while the button is held
   */
  public Command collectWhilePressedCommand(){
    return startEnd(() -> m_motor.set(MOTOR_SPEED), () -> m_motor.stopMotor());
  }


  /**
   * moves the coral to the edge of the cannon
   */
  public Command adjustCoralCommand(){
    return startEnd(() -> m_motor.set(ADJUST_SPEED),() -> m_motor.stopMotor()).until(() -> !m_sensor.get());
  }


  /**
   * releases the coral
   */
  public Command loosenCoralCommand(){
    return startEnd(() -> m_motor.set(MOTOR_SPEED),() -> m_motor.stopMotor()).withTimeout(LOOSEN_TIME);
  }


  private void configs(){
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CURRENT_PEAK).
    idleMode(NEUTRAL_MODE).inverted(INVERTED);
    m_motor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Sensor", m_sensor.get());
  }


}
