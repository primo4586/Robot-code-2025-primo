// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Disposer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Cannon.CannonConstants.MOTOR_SPEED;
import static frc.robot.subsystems.Disposer.DisposerConstants.*;

public class Disposer extends SubsystemBase {
  private SparkMax _motor;
  private RelativeEncoder _encoder;
  private PIDController _pidController;

  // singelton
  private static Disposer instance;

  /**
   * Gets the instance of Disposer. This is the only way to get an
   * instance of the class, as the constructor is private.
   *
   * @return the instance of Disposer
   */
  public static Disposer getInstance() {
    if (instance == null) {
      instance = new Disposer();
    }
    return instance;
  }

  private Disposer() {
    _motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    _encoder = _motor.getAlternateEncoder();
    _encoder.setPosition(0);
    _pidController = new PIDController(KP, KD, KS);
    _pidController.setSetpoint(0);
    _pidController.setTolerance(MINIMUN_POSITION_ERROR);
    configs();
  }

  /**
   * A command that moves the disposer to a given position.
   * this command is without the pid controller and over shoot the given position
   *
   * @param position the position to move to.
   * @return a command that moves the disposer to the given position.
   */
  public Command moveCommand(double position) {
    return startEnd(() -> _motor.setVoltage(_encoder.getPosition() < position ? MOTOR_SPEED : -MOTOR_SPEED),
        () -> _motor.setVoltage(0))
        .until(() -> Math.abs(_encoder.getPosition() - position) < MINIMUN_POSITION_ERROR);
  }

  /**
   * A command that moves the disposer to a given position using a PID
   * controller. This command is more accurate than the moveCommand and will
   * not over shoot the given position.
   *
   * @param position the position to move to.
   * @return a command that moves the disposer to the given position using a
   *         PID controller.
   */
  public Command moveWithPIDCommand(double position){
    return startEnd(() -> _motor.setVoltage(_pidController.calculate(_encoder.getPosition(), position)),
    () -> _motor.set(0)
    ).until(() -> _pidController.atSetpoint());
  }

  /**
   * Resets the encoder position to 0.
   *
   * @param position - the position to set the encoder to. ignored.
   * @return a command that resets the encoder position to 0.
   */
  public Command resetPositionCommand(double position) {
    return runOnce(() ->_encoder.setPosition(0));
  }

  @Override
  public void periodic() {
  }

  private void configs(){
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(PEAK_CURRENT).
    idleMode(NEUTRAL_MODE).inverted(INVERTED);
    _motor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
}