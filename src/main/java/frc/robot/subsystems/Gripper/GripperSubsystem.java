// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.subsystems.Gripper.GripperConstants.*;
import static frc.robot.Misc.*;

public class GripperSubsystem extends SubsystemBase {
  private TalonFX m_motor;
  private boolean velocityCheck = false;
  private double timer = 0;

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
   * checks if the algea is collected
   * @return true if the algea is collected
   */
  private boolean hasAlgea(){
    if (m_motor.getVelocity().getValueAsDouble() > VELOCITY_MIN_ERROR) // if the motor pass the velocity threshold
      velocityCheck = true; 
      
    if (velocityCheck && Math.abs(m_motor.getVelocity().getValueAsDouble()) < VELOCITY_MIN_ERROR)
      timer += 0.02;
      
    return  timer > HOLED_TIME; // if the motor started slowing down 
  }

  /**
   * collects the algea while pressing the joysticks, and when release stops the motor. 
   */
  public Command collectWhilePressCommand(){
      return startEnd(() -> m_motor.set(COLLECT_POWER),
      () -> m_motor.set(HOLED_POWER));
  }

  /**
   * Collects the algea until it is collected.
   * It first collect with power {@link #COLLECT_POWER}, and hold the algea with power {@link #HOLED_POWER},
   * and waits for {@link #HOLED_TIME} seconds to make sure the algea is collected.
   * @return a command that collects the algea until it is collected
   */
  public Command collectUntilCollectedCommand(){ 
    return startEnd(() -> m_motor.set(COLLECT_POWER),
    () ->
     {m_motor.set(HOLED_POWER);
      velocityCheck = false; // reset the velocity check
      timer = 0; // reset the timer 
     })
    .until(() -> hasAlgea()); // this checks if once the motor was at high speed now the motor is slow.
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
    return runEnd(() -> m_motor.set(TOSS_POWER),
    ()-> m_motor.stopMotor()).withTimeout(TOSS_TIME);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Gripper/has algea", hasAlgea());
    SmartDashboard.putBoolean("Gripper/Velocity check", velocityCheck);
    SmartDashboard.putNumber("Gripper/Timer", timer);

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
