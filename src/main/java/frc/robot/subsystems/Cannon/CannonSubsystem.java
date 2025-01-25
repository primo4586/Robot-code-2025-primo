package frc.robot.subsystems.Cannon;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Misc.*;
import static frc.robot.subsystems.Cannon.CannonConstants.*;


public class CannonSubsystem extends SubsystemBase {

  private TalonFX m_motor;
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
    m_motor = new TalonFX(MOTOR_ID, CANIVOR_NAME);
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
    TalonFXConfiguration configuration = new TalonFXConfiguration();
   
    //Peaks:
    configuration.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    configuration.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

    //settings
    configuration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    configuration.MotorOutput.Inverted = INVERTED;

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 3; i++){
      m_motor.getConfigurator().apply(configuration);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("canon config failed");
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Sensor", m_sensor.get());
  }


}
