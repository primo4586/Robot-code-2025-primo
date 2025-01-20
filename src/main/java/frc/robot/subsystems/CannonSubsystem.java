package frc.robot.subsystems;


import java.lang.module.Configuration;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CannonConstants;
import frc.robot.Constants.CannonConstants;
import frc.robot.Constants.RobotConstants;


public class CannonSubsystem extends SubsystemBase {


  private TalonFX m_motor;
  private DigitalInput m_sensor;
  //private VoltageOut voltageOut = new VoltageOut(0);


  //singlton
  private static CannonSubsystem instance;


  public static CannonSubsystem getInstance(){
    if (instance == null){
      instance = new CannonSubsystem();
    }
    return instance;
  }


  private CannonSubsystem(){
    m_motor = new TalonFX(CannonConstants.MOTOR_ID, RobotConstants.Canivore);
    m_sensor = new DigitalInput(CannonConstants.SENSOR_ID);
    configs();
  }


  /**
   * collects until the sensor is true
   */
  public Command collectUntilCoralCommand(){
    return startEnd(() -> m_motor.set(CannonConstants.MOTOR_SPEED),() -> m_motor.stopMotor()).until(() -> m_sensor.get());
  }


  /**
   * collects while the button is held
   */
  public Command collectWhilePressedCommand(){
    return startEnd(() -> m_motor.set(CannonConstants.MOTOR_SPEED), () -> m_motor.stopMotor());
  }


  /**
   * moves the coral to the edge of the cannon
   */
  public Command adjustCoralCommand(){
    return startEnd(() -> m_motor.set(CannonConstants.ADJUST_SPEED),() -> m_motor.stopMotor()).until(() -> !m_sensor.get());
  }


  /**
   * releases the coral
   */
  public Command loosenCoralCommand(){
    return startEnd(() -> m_motor.set(CannonConstants.MOTOR_SPEED),() -> m_motor.stopMotor()).withTimeout(CannonConstants.LOOSEN_TIME);
  }


  private void configs(){
    TalonFXConfiguration configuration = new TalonFXConfiguration();
   
    //Peaks:
    configuration.Voltage.PeakForwardVoltage = CannonConstants.PEAK_FORWARD_VOLTAGE;
    configuration.Voltage.PeakReverseVoltage = CannonConstants.PEAK_REVERSE_VOLTAGE;

    //settings
    configuration.MotorOutput.NeutralMode = CannonConstants.NEUTRAL_MODE;
    configuration.MotorOutput.Inverted = CannonConstants.INVERTED;
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Sensor", m_sensor.get());
      // TODO Auto-generated method stub
      super.periodic();
  }


}
