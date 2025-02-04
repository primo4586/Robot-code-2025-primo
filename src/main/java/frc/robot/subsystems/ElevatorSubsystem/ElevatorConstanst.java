package frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstanst { // todo: tune values 
    public static final int MASTER_TALONFX_ID = 0;
    public static final int FOLLOW_TALONFX_ID = 1;

    public static final double ROLLER_RADIUS = 0.1; // In meters
    public static final double MAXIMUM_HIGHT = 0; // In meters
    public static final double GEAR_RATIO = 27 ;  // todo: i'm not sure if it true 

    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {

      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 60; //^ i'm not sure if it true
      ELEVATOR_CONFIG.Voltage.PeakForwardVoltage = 11.5;
      ELEVATOR_CONFIG.Voltage.PeakReverseVoltage = -11.5;


      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_HIGHT; 
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO / (ROLLER_RADIUS * 2 * Math.PI); 

      ELEVATOR_CONFIG.Slot0.kG = 0.3; // Volts to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 0.4; // Volts to overcome static friction
      ELEVATOR_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
      ELEVATOR_CONFIG.Slot0.kP = 0.3;
      ELEVATOR_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_CONFIG.Slot0.kD = 0.0;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 350;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 2500;      
    }

    public static final double HIGHT_OF_THE_GROUND = 0;
    public static final double MINIMUN_POSITION_ERROR = 0.5;
    public static final double MINIMUN_VELOCITY_ERROR = 0.5;

    public static final double MOVE_POWER = 0.3;

    public static final double L1_Hight = 0;
    public static final double L2_Hight = 0;
    public static final double L3_Hight = 0;
    public static final double L4_Hight = 0; // todo tune values

}
