// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.Gripper.GripperConstants.*;
import static frc.robot.Misc.*;

public class GripperSubsystem extends SubsystemBase {
  private TalonFX m_motor;

  // singelton
  private static GripperSubsystem instance;
  public static GripperSubsystem getInstance(){
      if (instance == null) {
          instance = new GripperSubsystem();
      }
      return instance;
  }

  private GripperSubsystem() {
      m_motor = new TalonFX(MOTOR_ID,CANIVOR_NAME);
      configs();
  }

  /**
   * collects the algea while pressing the joysticks, and when release stops the motor. 
   */
  public Command collectWhilePressCommand(){
      return startEnd(() -> m_motor.set(COLLECT_POWER),
      () -> m_motor.set(HOLED_POWER));
  }

  /**
   * stops the motor 
   */
  public Command lossenGripCommand(){
    return runOnce(() -> m_motor.stopMotor());
  }
  
  /**
   * tossing the algea and then stops the 
   */
  public Command tossCommand() {
    return runOnce(() -> m_motor.set(TOSS_POWER)).withTimeout(TOSS_TIME);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  private void configs(){
     TalonFXConfiguration configuration = new TalonFXConfiguration();
   
    //Peaks:
    configuration.Voltage.PeakForwardVoltage = VOLTAGE_PEAK;
    configuration.Voltage.PeakReverseVoltage = VOLTAGE_PEAK * -1; //^ this may not be prety but it works

    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = CURRENT_PEAK;


    //settings
    configuration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    configuration.MotorOutput.Inverted = INVERTED;

    //upload configs motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      statusCode = m_motor.getConfigurator().apply(configuration);
      if(statusCode.isOK())
      break;
    }
    if (!statusCode.isOK())
    System.out.println("Gripper could not apply config, error code:" + statusCode.toString());
  }

}
