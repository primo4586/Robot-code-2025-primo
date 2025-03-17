// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.subsystems.Disposer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static frc.robot.subsystems.Disposer.DisposerConstants.*;

import java.security.PublicKey;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Misc.*;
public class Disposer extends SubsystemBase {

  private TalonFX m_motor; // falcon 500 motor
  private DigitalInput m_limitSwitch; // limit switch idk ISR really wanted it
  private CommandXboxController joyStick = RobotContainer._operatorController;

  private double lastAngle = 0; 
  private Double angle = 0.0;
  
  private boolean isDiffrent = false;
  /*
   * we dont use mm beause the movment of the arm is minimall and mm is for large distance movment.
   */
  private PositionVoltage systemControl; // to control a position

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
          null ,        // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null,        // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
          (volts) -> m_motor.setControl(m_voltReq.withOutput(volts.in(Volts))),
          null,
          this
        )
    );


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
    m_motor = new TalonFX(MOTOR_ID, CANIVOR_NAME);
    m_limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
    systemControl = new PositionVoltage(0);
    resetPosition();
    configs();

  }


  public void resetPosition() {
    m_motor.setPosition(0);
    lastAngle = 0;
  }

  /**
   * check if the disposer arm is at the set point.
   * the method use the absolute difference between the set point and the current position
   * and the absolute value of the current velocity to determine if the arm is at the set point or not.
   * @return true if the arm is at the set point. false otherwise.
   */
  private boolean isAtSetPoint() {
    return Math.abs(m_motor.getPosition().getValueAsDouble() -  (lastAngle)) < MINIMUN_POSITION_ERROR &&
     m_motor.getVelocity().getValueAsDouble() < MINIMUN_VELOCITY;
  }

  public void getAngle() {
    
    angle  = joyStick.getRightY() > 0.7 ? REEF_ANGLE :
            joyStick.getRightY() < -0.7 ? FLOOR_ANGLE :
            joyStick.getRightX() < -0.7 ? PROCESSOR_ANGLE:
            lastAngle;
    
    if (lastAngle != angle){
      isDiffrent = true;
      lastAngle = angle;
    }else{
      isDiffrent = false;
    }

  }

  public boolean isDiffrent(CommandXboxController joyStick){
    getAngle();
    return lastAngle != angle;
  }

  public Command setAngle(double angle){
      return runOnce(() -> lastAngle = angle);  
    }

  /**
   * a command that will bring the disposer arm to the home position, (up position).
   * the command will run until the switch is pressed.
   * @return a command that will bring the disposer arm to the home position.
   */ 
  public Command setHomeCommand() {
    return startEnd(() -> m_motor.set(RESET_POWER),
     () -> {
      resetPosition();
      lastAngle = 0;
      angle = 0.0;
      m_motor.set(0);

     }).until(() -> !m_limitSwitch.get());
  }



  /**
   * a command that will move the disposer arm to the target angle
   * you need to call it once per game. and use setTargetAngel to Change the target angle
   * @return
   */
  public Command relocateAngelCommand() {
      return runOnce(() -> {
        m_motor.setControl(systemControl.withPosition(angle));
      }).withName("Relocate arm to " + angle)
     .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
  public Command relocateAngelCommand(double angle) {
    return runOnce(() -> {
      this.angle = angle; 
      m_motor.setControl(systemControl.withPosition(angle));
    }).withName("Relocate arm to " + angle)
   .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
}

  public Command moveArmCommand(int vec){
    return startEnd(() -> m_motor.set(0.3 * vec), () -> lastAngle = m_motor.getPosition().getValueAsDouble() );
  }

  //sysId
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
 }
 
 public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
 }

  @Override
  public void periodic() {
    getAngle();
    SmartDashboard.putNumber("disposer/disposer position", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("disposer/is at targer", isAtSetPoint());
    SmartDashboard.putBoolean("disposer/disposer switch", m_limitSwitch.get());
    SmartDashboard.putNumber("disposer/target position", lastAngle);
    SmartDashboard.putBoolean("disposer/diffrent", isDiffrent);
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
       System.out.println("Disposer Config Failed:" + statusCode.toString());
  }
}