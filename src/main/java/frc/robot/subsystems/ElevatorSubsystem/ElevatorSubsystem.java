// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSubsystem;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.*;



public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_masterTalonFX;
  private TalonFX m_followTalonFX;
  private DigitalInput digitalSwitch;
  private static ElevatorSubsystem instance;
  /** Creates a new ElevatorSubsystem. */


  //instance:
  public static ElevatorSubsystem getInstance(){
    if (instance == null)
      instance = new ElevatorSubsystem();
    return instance;
  }

  //TODO: set follow motor to  actually follow

  private ElevatorSubsystem()
   {
    //configs();
    //initing:
    m_masterTalonFX = new TalonFX(Constants.Elevator.PRIMARY_TALONFX_ID, Constants.Elevator.CAN_BUS_NAME);
    m_followTalonFX = new TalonFX(Constants.Elevator.FOLLOW_TALONFX_ID, Constants.Elevator.CAN_BUS_NAME);
    digitalSwitch = new DigitalInput(Constants.Elevator.DIGITAL_SWITCH_ID);

    m_followTalonFX.follow(m_masterTalonFX);  

  }

  

  //reseting pos
  public Command resetPosition(){
    return run(()-> {
      m_masterTalonFX.setPosition(0);
    m_followTalonFX.setPosition(0);
  });
  }


  //hCommand -  home command, takes target position and moves to that position
  public Command hCommand(double targetPos){
    return run(()-> {
      if(digitalSwitch.get()){ //if difital switch is being pressed
        m_masterTalonFX.setPosition(0);  

        //TODO: add follow and then u dont need this shit because it follows
        m_followTalonFX.setPosition(targetPos); 
    }
    });
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private void configs(){
    
  }
}
