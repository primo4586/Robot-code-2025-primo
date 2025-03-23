package frc.robot.subsystems.Disposer;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Misc;

import static frc.robot.subsystems.Disposer.DisposerConstants.*;
public class Disposer extends SubsystemBase {

  private TalonFX m_motor; // falcon 500 motor
  private PositionVoltage systemControl; // to control a position



  // singelton
  private static Disposer instance;

  /**
   * Get the single instance of the Disposer.
   * This class is a singleton, so there is only one instance of it. This
   * method is used to get that instance.
   *
   * @return the single instance of the Disposer
   */
  public static Disposer getInstance() {
    if (instance == null) {
      instance = new Disposer();
    }
    return instance;
  }

  private Disposer() {
    m_motor = new TalonFX(MOTOR_ID);
    systemControl = new PositionVoltage(0);
    resetPosition();
    configs();

  }


  public void resetPosition() {
    m_motor.setPosition(0);
  }

  public Command preparingCommand() {
    return runOnce(() -> m_motor.setControl(systemControl.withPosition(READY_POSITION).withVelocity(2)));
  }

  public Command goHomeCommand() {
    return runOnce(() -> m_motor.setControl(systemControl.withPosition(HOME_POSITION).withVelocity(2)));
  }


  @Override
  public void periodic() {
  }

  private void configs() {
     TalonFXConfiguration configuration = new TalonFXConfiguration();
 
     //Slot:
     configuration.Slot0.kP = KP;
     configuration.Slot0.kD = KD;
     configuration.Slot0.kV = KV;
     configuration.Slot0.kS = KS;
 
   //Peeks:
     configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
     configuration.CurrentLimits.SupplyCurrentLimit = PEAK_CURRENT;

     configuration.Voltage.PeakForwardVoltage = PEAK_VOLTAGE;
     configuration.Voltage.PeakReverseVoltage = PEAK_VOLTAGE * -1;
    
 
     // forward and backward limits 
     configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
     configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FOWORD_LIMIT;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BACKWARD_LIMIT;

     configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
     configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
     StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
 
 
     //upload configs to motor
     for (int i = 0; i < 5; i++){
       statusCode = m_motor.getConfigurator().apply(configuration);
       if (statusCode.isOK())
         break;
     }
     if (!statusCode.isOK())
       System.out.println("Disposer Config Failed:" + statusCode.toString());
  }
}