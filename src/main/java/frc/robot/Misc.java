package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Misc {
    public static final String CANIVOR_NAME = "canBus";

    //swerve Commands
    //^the Rotation2d is false bit it is uslless
    public static final Pose2d BLUE_REEF_CENTER_POSITION = new Pose2d (4.5,4,new Rotation2d()); 
    public static final Pose2d RED_REEF_CENTER_POSITION = new Pose2d (13,4,new Rotation2d()); 

    public static final double REEF_1_ANGLE = 180; //^there is a good chance i am wrong
    public static final double REEF_2_ANGLE = 120;
    public static final double REEF_3_ANGLE = 60;
    public static final double REEF_4_ANGLE = 0;
    public static final double REEF_5_ANGLE = 300;
    public static final double REEF_6_ANGLE = 240;

    public static final Pose2d BLUE_REEF_A_POSITION = new Pose2d(3.15,4.01,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_B_POSITION = new Pose2d(3.15,3.77,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_C_POSITION = new Pose2d(3.77,3.17,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_D_POSITION = new Pose2d(4.04,2.74,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_E_POSITION = new Pose2d(5.10,2.85,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_F_POSITION = new Pose2d(5.38,3.01,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_G_POSITION = new Pose2d(5.80,3.95,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_H_POSITION = new Pose2d(5.80,4.28,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_I_POSITION = new Pose2d(5.18, 5.11,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_J_POSITION = new Pose2d(4.94,5.29,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_K_POSITION = new Pose2d(3.88,5.22,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d BLUE_REEF_L_POSITION = new Pose2d(3.57,5.04,new Rotation2d(Math.toRadians(-60)));

    public static final Pose2d RED_REEF_A_POSITION = new Pose2d(14.260,3.885,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_B_POSITION = new Pose2d(14.301,4.315,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_C_POSITION = new Pose2d(13.733,5.015,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_D_POSITION = new Pose2d(13.424,5.275,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_E_POSITION = new Pose2d(12.522,5.081,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_F_POSITION = new Pose2d(12.191,4.978,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_G_POSITION = new Pose2d(11.844,4.127,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_H_POSITION = new Pose2d(11.802,3.750,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_I_POSITION = new Pose2d(12.46,3.22,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_J_POSITION = new Pose2d(12.75,2.98,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_K_POSITION = new Pose2d( 13.632,2.968,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d RED_REEF_L_POSITION = new Pose2d(13.953,3.084,new Rotation2d(Math.toRadians(120)));
}
