package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstanst { 
    public static final int MASTER_TALONFX_ID = 30;
    public static final int FOLLOW_TALONFX_ID = 31;

    public static final double ROLLER_RADIUS = 0.048; // In meters
    public static final double MAXIMUM_HIGHT = 2.516845703125; // In meters
    public static final double GEAR_RATIO = 15 ;

    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {

      ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
      ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 80; //^ at 60 (amp) it was to slow and use max amp. i dont know if you need cuurent limit here
      ELEVATOR_CONFIG.Voltage.PeakForwardVoltage = 11.5;
      ELEVATOR_CONFIG.Voltage.PeakReverseVoltage = -11.5;


      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_HIGHT; 
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -2;

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO / (ROLLER_RADIUS * 2 * Math.PI);

      ELEVATOR_CONFIG.Slot0.kG = 3; // Current to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 17; // Current to overcome static friction
      //^ when using MotionMagicTorqueCurrentFOC you dont kV and kA
      // ELEVATOR_CONFIG.Slot0.kV = 3.0932; // Volts for a velocity target of 1 rps
      // ELEVATOR_CONFIG.Slot0.kA = 0.75; // Volts for an acceleration of 1 rps/s 
      ELEVATOR_CONFIG.Slot0.kP = 100;
      ELEVATOR_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_CONFIG.Slot0.kD = 130;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 10;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 3 ;      
    }

    public static final double HIGHT_OF_THE_GROUND = 0;
    public static final double MINIMUN_POSITION_ERROR = 0.08;
    public static final double MINIMUN_VELOCITY_ERROR = 0.08;

    public static final double MOVE_POWER = 0.3;

    public static final double L1_HEIGHT = 0;
    public static final double L2_HEIGHT = 0.703857421875;
    public static final double L3_HEIGHT = 1.390380859375;
    public static final double L4_HEIGHT = 2.441650390625; 
    //our reef height: 2.553173828125 - 0.10;
    //real reef height: 2.441650390625; 

    public static final double LOW_ALGEA_HEIGHT = 0.89;
    public static final double HIGH_ALGEA_HEIGHT = 0;

}
