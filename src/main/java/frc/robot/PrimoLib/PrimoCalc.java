package frc.robot.PrimoLib;

import static frc.robot.Misc.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PrimoCalc {
     private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
             /**
         * Calculates the angle between the robot's current position and a target point.
         * 
         * The angle is calculated using the current pose of the robot and the target point.
         * The angle is then adjusted to be relative to the robot's current heading, so a
         * positive angle will result in a counter-clockwise rotation of the robot, and a
         * negative angle will result in a clockwise rotation.
         * 
         * @param targetPose The target point to calculate the angle to.
         * @return The angle between the robot's current position and the target point.
         */
        public static Rotation2d calculateAngleToPoint(Translation2d targetPose) {

                Pose2d currentPose = swerve.getState().Pose; // todo camera?

                // Get the difference in x and y positions
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();

                // Use atan2 to calculate the angle between the two points
                double angleToTargetRadians = Math.atan2(dy, dx);

                // Return the angle as a Rotation2d
                return new Rotation2d(angleToTargetRadians + Units.degreesToRadians(180));
        }

        //TODO: make this work for red alliance too
        public static Pose2d ChooseReef(boolean isRight){ 

                Pose2d currentPosition = swerve.getState().Pose;
                BooleanSupplier isBlue = () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
                /*
                 * caculate the angel between the robot and the reef 
                 * from here we can know to wich side of the reef the robot needs to go.
                */

                if (isBlue.getAsBoolean()){
                        double reefAngle = 
                        Math.toDegrees(Math.atan(
                       (BLUE_REEF_CENTER_POSITION.getY() - currentPosition.getY()) /
                       (BLUE_REEF_CENTER_POSITION.getX() - currentPosition.getX())));

               if (-90 < reefAngle && reefAngle <= -30){
                       if (currentPosition.getX() > BLUE_REEF_CENTER_POSITION.getX()){
                               return new Pose2d(
                                       isRight ? BLUE_REEF_F_POSITION.getX() : BLUE_REEF_E_POSITION.getX(),
                                       isRight ? BLUE_REEF_F_POSITION.getY() : BLUE_REEF_E_POSITION.getY(),
                                       new Rotation2d(REEF_3_ANGLE));
                       }
                       else
                       {
                               return new Pose2d(
                               isRight ? BLUE_REEF_L_POSITION.getX() : BLUE_REEF_K_POSITION.getX(),
                               isRight ? BLUE_REEF_L_POSITION.getY() : BLUE_REEF_K_POSITION.getY(),
                               new Rotation2d(REEF_6_ANGLE));
                       }
               }
               else if (reefAngle <= 30){
                        if (currentPosition.getX() > BLUE_REEF_CENTER_POSITION.getX())
                       {
                               return new Pose2d(
                               isRight ? BLUE_REEF_H_POSITION.getX() : BLUE_REEF_G_POSITION.getX(),
                               isRight ? BLUE_REEF_H_POSITION.getY() : BLUE_REEF_G_POSITION.getY(),
                               new Rotation2d(REEF_4_ANGLE));
                       }
                       else
                       {
                               return new Pose2d(
                               isRight ? BLUE_REEF_B_POSITION.getX() : BLUE_REEF_A_POSITION.getX(),
                               isRight ? BLUE_REEF_B_POSITION.getY() : BLUE_REEF_A_POSITION.getY(),
                               new Rotation2d(REEF_1_ANGLE));
                       }
                }
               else{
                       if (currentPosition.getX() > BLUE_REEF_CENTER_POSITION.getX())
                       {
                               return new Pose2d(
                               isRight ? BLUE_REEF_H_POSITION.getX() : BLUE_REEF_G_POSITION.getX(),
                               isRight ? BLUE_REEF_H_POSITION.getY() : BLUE_REEF_G_POSITION.getY(),
                               new Rotation2d(REEF_5_ANGLE));
                       }
                       else
                       {
                               return new Pose2d(
                               isRight ? BLUE_REEF_D_POSITION.getX() : BLUE_REEF_C_POSITION.getX(),
                               isRight ? BLUE_REEF_D_POSITION.getY() : BLUE_REEF_C_POSITION.getY(),
                               new Rotation2d(REEF_2_ANGLE));
                       }        
                }
                }
                else{
                double reefAngle = 
                Math.toDegrees(Math.atan(
                (RED_REEF_CENTER_POSITION.getY() - currentPosition.getY()) /
                (RED_REEF_CENTER_POSITION.getX() - currentPosition.getX())));
         
                if (-90 < reefAngle && reefAngle <= -30){
                       if (currentPosition.getX() < RED_REEF_CENTER_POSITION.getX()){
                               return new Pose2d(
                                       isRight ? RED_REEF_L_POSITION.getX() : RED_REEF_K_POSITION.getX(),
                                       isRight ? RED_REEF_L_POSITION.getY() : RED_REEF_K_POSITION.getY(),
                                       new Rotation2d(REEF_3_ANGLE));
                       }
                       else
                       {
                               return new Pose2d(
                               isRight ? RED_REEF_F_POSITION.getX() : RED_REEF_E_POSITION.getX(),
                               isRight ? RED_REEF_F_POSITION.getY() : RED_REEF_E_POSITION.getY(),
                               new Rotation2d(REEF_6_ANGLE));
                       }
               }
               else if (reefAngle <= 30){
                        if (currentPosition.getX() < RED_REEF_CENTER_POSITION.getX())
                       {
                               return new Pose2d(
                               isRight ? RED_REEF_B_POSITION.getX() : RED_REEF_A_POSITION.getX(),
                               isRight ? RED_REEF_B_POSITION.getY() : RED_REEF_A_POSITION.getY(),
                               new Rotation2d(REEF_4_ANGLE));
                       }
                       else
                       {
                               return new Pose2d(
                               isRight ? RED_REEF_H_POSITION.getX() : RED_REEF_G_POSITION.getX(),
                               isRight ? RED_REEF_H_POSITION.getY() : RED_REEF_G_POSITION.getY(),
                               new Rotation2d(REEF_1_ANGLE));
                       }
               }
               else{
                       if (currentPosition.getX() < RED_REEF_CENTER_POSITION.getX())
                       {
                               return new Pose2d(
                               isRight ? RED_REEF_D_POSITION.getX() : RED_REEF_C_POSITION.getX(),
                               isRight ? RED_REEF_D_POSITION.getY() : RED_REEF_C_POSITION.getY(),
                               new Rotation2d(REEF_5_ANGLE));
                       }
                       else
                       {
                               return new Pose2d(
                               isRight ? RED_REEF_J_POSITION.getX() : RED_REEF_I_POSITION.getX(),
                               isRight ? RED_REEF_J_POSITION.getY() : RED_REEF_I_POSITION.getY(),
                               new Rotation2d(REEF_2_ANGLE));
                       }        
               }
        }

       
                
        }
}
