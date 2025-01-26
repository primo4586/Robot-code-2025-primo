// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.ControlModeValue;

import com.ctre.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

import java.lang.module.Configuration;
import java.time.Duration;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.ctre.phoenix6.configs.TalonFXConfiguration;


import com.ctre.phoenix6.controls.*;
import frc.robot.Constants.Elevator.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.controls.*;



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


    private ElevatorSubsystem()
   {
    //initing:
    configs();
    m_masterTalonFX = new TalonFX(Elevator.MASTER_TALONFX_ID);
    m_followTalonFX = new TalonFX(Elevator.FOLLOW_TALONFX_ID);
    digitalSwitch = new DigitalInput(Elevator.DIGITAL_SWITCH_ID);

    m_followTalonFX.setInverted(false);
    m_masterTalonFX.setInverted(false);
    resetPosition();
    

  }


  //reseting pos
  public Command resetPosition(){
    return runOnce(()-> {
      m_masterTalonFX.setPosition(0);
      m_followTalonFX.setPosition(0);

  });
  }

  
  
  /**
   * @param targetPos
   *  takes target position and moves to that position
   */
  public Command homeCommand(double targetPos) {
    return runEnd(
        () -> {
            if (digitalSwitch.get()) { //if the digital switch is pressed
                resetPosition(); 
            }
        },
        () -> {
          //stop both motors
            m_followTalonFX.stopMotor(); 
            m_masterTalonFX.stopMotor();  
        }
    );
}



  //relocate to a given position using Motion Magic
  public Command relocatePositionCommand(double targetPos) {
      return runOnce(() -> {
        PositionVoltage m_request = new PositionVoltage(targetPos).withSlot(0);
        m_masterTalonFX.setControl(m_request); // Engage Motion Magic control mode
        m_masterTalonFX.set(targetPos);  // Set the target position for the Motion Magic
        m_followTalonFX.set(Constants.Elevator.MASTER_TALONFX_ID);    
        });
  }


  @Override
  public void periodic() {
    double currentPosition = m_masterTalonFX.getPosition().getValueAsDouble();
          
    //TODO: put target pos
      double targetPosition = 0;
      if (Math.abs(currentPosition - targetPosition) < 10) {
          System.out.println("Elevator is near target position!");
      }
  
      if (Math.abs(currentPosition - targetPosition) < 10) {
          System.out.println("reached target!");
      }
  }
  
  private void configs(){
    TalonFXConfiguration  configuration = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();
    
    mm.MotionMagicCruiseVelocity = Elevator.MM_CRUISE; //maximun velocity (peak velocity of the motion)
    mm.MotionMagicAcceleration = Elevator.MM_ACCELERATION; //controls acceleration + deceleration (beginning and end of motion)
    mm.MotionMagicJerk = Elevator.MM_JERK; //jerk - derivative of acceleration

    configuration.MotionMagic = mm; 

  /*
    p - proportional gain, controls the system's response to the current error in position
    i - integral gain, controls the system's response to collected errors over time
    d - derivative gain, controls how fast the position is changing
    s - static friction gain - compensates for static friction (ensures that the motor has enough output to initiate motion)
    v - velocity gain - controls the motor output based on the velocity of the elevator
    a - acceleration gain - controls the motor output based on the acceleration
    g - gravity - output to overcome gravity
   */

    configuration.Slot0.kP = Constants.Elevator.kP;
    configuration.Slot0.kI = Constants.Elevator.kI;
    configuration.Slot0.kD = Constants.Elevator.kD;
    configuration.Slot0.kS = Constants.Elevator.kS;
    configuration.Slot0.kA = Constants.Elevator.kA;
    configuration.Slot0.kV = Constants.Elevator.kV;
    configuration.Slot0.kG = Constants.Elevator.kG;


    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; i++){
      statusCode = m_masterTalonFX.getConfigurator().apply(configuration);
      if(statusCode.isOK())
        break;
    }
    if(!statusCode.isOK())
      System.out.println("master elevator motor cant apply config, error code: " + statusCode.toString());

    for(int i = 0; i < 5; i++){
      statusCode = m_followTalonFX.getConfigurator().apply(configuration);
      if(statusCode.isOK())
        break;
    }
    if(!statusCode.isOK())
      System.out.println("follow elevator motor cant apply config, error code: " + statusCode.toString());
    



    
  }
}
