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
       public static Command alignPointCommand(Translation2d targetPoint) { // todo connect cameras
        // todo add all 8 reef 2d points. 

                PIDController pidController = new PIDController(0.0, 0.0, 0); // todo tune
                pidController.enableContinuousInput(-180, 180);
                pidController.setTolerance(0.5);
                pidController.setSetpoint(0);

                return new RunCommand(
                                () -> swerve.setControl(
                                                forwardStraight.withRotationalRate(
                                                                pidController.calculate(
                                                                                swerve.getState().Pose.getRotation() // todo: switch with camera
                                                                                                .getDegrees(),
                                                                                calculateAngleToPoint(targetPoint)
                                                                                                .getDegrees())))); // if Tolerance dose not work 
                                                                                                //,swerve).until(() -> pidController.atSetpoint()).andThen(() -> pidController.close());
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

    
}
