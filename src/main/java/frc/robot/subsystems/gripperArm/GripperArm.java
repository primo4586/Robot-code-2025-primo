// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripperArm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.gripperArm.GripperArmConstants.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Misc.*;
public class GripperArm extends SubsystemBase {

  private TalonFX m_motor; // falcon 500 motor
  private DigitalInput m_limitSwitch; // limit switch idk ISR really wanted it
  private TalonSRX m_encoder; // magCoder
  
  private static DoubleSupplier targetAngel = () -> 0;
  /*
   * we dont use mm beause the movment of the arm is minimall and mm is for large distance movment.
   */
  private PositionVoltage systemControl; // to control a position

  // singelton
  private static GripperArm instance;

  /**
   * Get the single instance of the GripperArm.
   *
   * <p>
   * This class is a singleton, so there is only one instance of it. This
   * method is used to get that instance.
   *
   * @return the single instance of the GripperArm
   */
  public static GripperArm getInstance() {
    if (instance == null) {
      instance = new GripperArm();
    }
    return instance;
  }

  private GripperArm() {
    m_motor = new TalonFX(MOTOR_ID, CANIVOR_NAME);
    m_encoder = new TalonSRX(ENCODER_ID);
    systemControl = new PositionVoltage(0);
    resetPosition();
    configs();

  }

  /**
   * reset the encode position value.
   * use this at the start of robotInit.
   */
  public void resetPosition() {
    m_motor.setPosition(5 * m_encoder.getSelectedSensorPosition() / 4096); // 5 is the gear ratio 
  }

  /**
   * check if the gripper arm is at the set point.
   * the method use the absolute difference between the set point and the current position
   * and the absolute value of the current velocity to determine if the arm is at the set point or not.
   * @return true if the arm is at the set point. false otherwise.
   */
  private boolean isAtSetPoint() {
    return Math.abs(m_motor.getPosition().getValueAsDouble() -  (targetAngel.getAsDouble() / 360)) < MINIMUN_POSITION_ERROR &&
     m_motor.getVelocity().getValueAsDouble() < MINIMUN_VELOCITY;
  }

  /**
   * a command that will bring the gripper arm to the home position, (up position).
   * the command will run until the switch is pressed.
   * @return a command that will bring the gripper arm to the home position.
   */ 
  public void setHomeCommand() {
    this.targetAngel = () -> 0;
  }

  /**
   * set the target angle
   * @param targetAngel
   */
  public void setTargetAngel(double targetAngel){ 
    this.targetAngel = () -> targetAngel;

  }
  /**
   * a command that will move the gripper arm to the target angle
   * you need to call it once per game. and use setTargetAngel to Change the target angle
   * @return
   */
  public Command relocateAngelCommand() {
      return run(() -> m_motor.setControl(systemControl.withPosition(targetAngel.getAsDouble())));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gripper position", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("is at targer", isAtSetPoint()); // todo: put this in a folder. 
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
    
     configuration.Feedback.SensorToMechanismRatio = GEAR_RATIO;
 
     // forward and backward limits 
     configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
     configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FOWORD_LIMIT;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BACKWARD_LIMIT;

     configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
     configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
     configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = LIMIT_SWITCH_POSITION;

     m_encoder.setSelectedSensorPosition(ABS_SENSOR_POSITION);
 
     configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
     configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
     StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
 
 
     //upload configs to motor
     for (int i = 0; i < 5; i++){
       statusCode = m_motor.getConfigurator().apply(configuration);
       if (statusCode.isOK())
         break;
     }
     if (!statusCode.isOK())
       System.out.println("Gripper Arm Config Failed:" + statusCode.toString());
  }
}