// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperArm extends SubsystemBase {
  /** Creates a new GripperArm. */

  private TalonFX m_motor;
  private DigitalInput _limitSwitch;
  private final MotionMagicExpoTorqueCurrentFOC mm = new MotionMagicExpoTorqueCurrentFOC(0);


  // singleton
  public static GripperArm instance;

  public static GripperArm getInstance()
  {
    if (instance == null){
      instance = new GripperArm();
    }
    return instance;
  }

  // constractor
  private GripperArm(){
    m_motor = new TalonFX(GripperArmConstans.MOTOR_ID, GripperArmConstans.CAN_BUS_NAME);
    _limitSwitch = new DigitalInput(GripperArmConstans.LIMIT_SWITCH);
  }

/**
    * Move the arm to a degree
    * @param degree
    * @return
    */
    public Command moveArmTo(double degree){
      return runOnce(() -> m_motor.setControl(mm.withPosition(degree)));
  
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
