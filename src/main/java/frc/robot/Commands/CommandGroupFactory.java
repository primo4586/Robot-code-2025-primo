package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Misc.*;

public class CommandGroupFactory {

    private static final CommandSwerveDrivetrain swerve = RobotContainer.drivetrain;
        private static final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

/**
 * Creates a command to align the robot's orientation towards a specified target point.
 * This command uses a PID controller to adjust the robot's rotational rate
 * to face the given target point in the field.
 * 
 * recemanded to use one camera for this and not odemetry
 *
 * @param targetPoint The 2D translation point that the robot should align to.
 * @return A command that aligns the robot's heading towards the target point.
 */
       public static Command alignPointCommand() { // todo connect cameras
        // todo add all 8 reef 2d points. 

                PIDController pidController = new PIDController(0.0, 0.0, 0); // todo tune
                pidController.enableContinuousInput(-180, 180);
                pidController.setTolerance(0.5);
                pidController.setSetpoint(0);
                return new RunCommand(
                                () -> swerve.setControl(
                                                forwardStraight.withRotationalRate(
                                                                pidController.calculate(5)))).// todo: connect to camera
                                                                        until(() -> pidController.atSetpoint());
        }

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

        //         public static Rotation2d calculateAngleToPass() {

        //         Pose2d currentPose = swerve.getState().Pose;
        //         Translation2d targetPose = Misc.passPosePoint;

        //         // Get the difference in x and y positions
        //         double dx = targetPose.getX() - currentPose.getX();
        //         double dy = targetPose.getY() - currentPose.getY();

        //         // Use atan2 to calculate the angle between the two points
        //         double angleToTargetRadians = Math.atan2(dy, dx);

        //         // Return the angle as a Rotation2d
        //         return new Rotation2d(angleToTargetRadians + Units.degreesToRadians(180));
        // }

        public static Pose2d ChooseReef(boolean isRight){ 

                Pose2d currentPosition = swerve.getState().Pose;
                /*
                 * caculate the angel between the robot and the reef 
                 * from here we can know to wich side of the reef the robot needs to go.
                */
                double reefAngle = Math.toDegrees(Math.atan(currentPosition.getY() - REEF_CENTER_POSITION.getY() /
                        currentPosition.getX() - REEF_CENTER_POSITION.getX()));
                reefAngle /= 60; 
                switch((int)reefAngle){
                        case 0: return new Pose2d(
                                isRight ? REEF_A_POSITION.getX() : REEF_B_POSITION.getX(),
                                isRight ? REEF_A_POSITION.getY() : REEF_B_POSITION.getY(),
                                new Rotation2d(REEF_1_ANGEL)
                        );
                        case 1: return new Pose2d(
                                isRight ? REEF_C_POSITION.getX() : REEF_D_POSITION.getX(),
                                isRight ? REEF_C_POSITION.getY() : REEF_D_POSITION.getY(),
                                new Rotation2d(REEF_2_ANGEL)
                        );
                        case 2: return new Pose2d(
                                isRight ? REEF_E_POSITION.getX() : REEF_F_POSITION.getX(),
                                isRight ? REEF_E_POSITION.getY() : REEF_F_POSITION.getY(),
                                new Rotation2d(REEF_3_ANGEL)
                        );
                        case 3: return new Pose2d(
                                isRight ? REEF_G_POSITION.getX() : REEF_H_POSITION.getX(),
                                isRight ? REEF_G_POSITION.getY() : REEF_H_POSITION.getY(),
                                new Rotation2d(REEF_4_ANGEL)
                        );
                        case 4: return new Pose2d(
                                isRight ? REEF_I_POSITION.getX() : REEF_J_POSITION.getX(),
                                isRight ? REEF_I_POSITION.getY() : REEF_J_POSITION.getY(),
                                new Rotation2d(REEF_5_ANGEL)
                        );
                        case 5: return new Pose2d(
                                isRight ? REEF_K_POSITION.getX() : REEF_L_POSITION.getX(),
                                isRight ? REEF_K_POSITION.getY() : REEF_L_POSITION.getY(),
                                new Rotation2d(REEF_6_ANGEL)
                        );
                        default: return new Pose2d();
                }
        
        }
}
