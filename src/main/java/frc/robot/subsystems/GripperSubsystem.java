// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.GripperConstants;

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
      m_motor = new TalonFX(GripperConstants.MOTOR_ID, RobotConstants.CANIVOR_NAME);
      configs();
  }

  /**
   * collects the algea while pressing the joysticks, and when release stops the motor. 
   */
  public Command collectWhilePressCommand(){
      return startEnd(() -> m_motor.set(GripperConstants.COLLECT_POWER),
      () -> m_motor.set(GripperConstants.HOLED_POWER));
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
    return runOnce(() -> m_motor.set(GripperConstants.TOSS_POWER)).withTimeout(GripperConstants.TOSS_TIME);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  private void configs(){
     TalonFXConfiguration configuration = new TalonFXConfiguration();
   
    //Peaks:
    configuration.Voltage.PeakForwardVoltage = GripperConstants.VOLTAGE_LIMIT;
    configuration.Voltage.PeakReverseVoltage = GripperConstants.VOLTAGE_LIMIT;


    //settings
    configuration.MotorOutput.NeutralMode = GripperConstants.NEUTRAL_MODE;
    configuration.MotorOutput.Inverted = GripperConstants.INVERTED;

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
