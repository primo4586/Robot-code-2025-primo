// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripperArm;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.gripperArm.GripperArmConstants.*;
import static frc.robot.Misc.*;
public class GripperArm extends SubsystemBase {

  private TalonFX m_motor; // falcon 500 motor
  private DigitalInput m_switch; // phisical switch
  /*
   * we dont use mm beause the movment of the arm is minimall and mm is for large distance movment.
   */
  private PositionVoltage systemControl; // to control a position
  // TODO: i'm not sure the a profile and Expo is good for this arm so i'll need
  // to check.

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
  public static GripperArm getInctance() {
    if (instance == null) {
      instance = new GripperArm();
    }
    return instance;
  }

  private GripperArm() {
    m_motor = new TalonFX(MOTOR_ID, CANIVOR_NAME);
    m_switch = new DigitalInput(SWITCH_ID);
    resetPosition();
    systemControl = new PositionVoltage(m_motor.get());
    configs();

  }

  /**
   * reset the encode position value.
   * use this at the start of robotInit.
   */
  public void resetPosition() {
    m_motor.setPosition(0);
  }

  /**
   * a command that will bring the gripper arm to the home position, (up position).
   * the command will run until the switch is pressed.
   * @return a command that will bring the gripper arm to the home position.
   */ 
  public Command setHomeCommand() {
    return startEnd(() -> m_motor.set(RESET_POWER),
        () -> {
          m_motor.stopMotor();
          resetPosition();
        }).until(() -> m_switch.get());
  }

  /**
   * a command that will bring the gripper arm to the given angle position.
   * the command will run until the angle is reached.
   * @param angle the angle the gripper arm should be moved to.
   * @return a command that will bring the gripper arm to the given angle position.
   */
  public Command relocateAngelCommand(Rotation2d angle) {
    return startEnd(() -> m_motor.setControl(systemControl.withPosition(360 / angle.getDegrees() * GEAR_RATIO)),
        () -> m_motor.stopMotor())
        .until(() -> Math.abs(
            m_motor.getPosition().getValueAsDouble() - (360 / angle.getDegrees())) < MINIMUN_ERROR);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("gripper switch", m_switch.get());
    SmartDashboard.putNumber("gripper position", m_motor.getPosition().getValueAsDouble());
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
     configuration.Voltage.PeakReverseVoltage = PEAK_VOLTAGE;
 
 
     // forward and backward limits 
     configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
     configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FOWORD_LIMIT;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BACKWARD_LIMIT;
     configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
     configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = LIMIT_SWITCH_POSITION;
 
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