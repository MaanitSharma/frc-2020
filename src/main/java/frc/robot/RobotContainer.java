/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DriveSubsystem;

//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  // The driver's controller
  public static final DriveSubsystem robotDrive = new DriveSubsystem();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /*robotDrive.setDefaultCommand( //getAutonomousCommand());
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> 
          robotDrive.justAFK()));*/
  }

  public Command hahahDoNothing() {
    System.out.println("hasdfjasd;flasd");
    Robot.driveTrain.driveSlave(0.1, 0.1);
    return null;
  }

  public Command getAutonomousCommand() {

    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            robotDrive.getFeedForward(),
            robotDrive.getKinematics(),
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(robotDrive.getKinematics())
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory Trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
       List.of(
            new Translation2d(1.0, 1.0),
            new Translation2d(2.0, -1.0)
        ),
        
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3.0, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        Trajectory,
        robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        robotDrive.getFeedForward(),
        robotDrive.getKinematics(),
        robotDrive::getWheelSpeeds,
        robotDrive.getLeftPIDController(),
        robotDrive.getRightPIDController(),
        // RamseteCommand passes volts to the callback
        robotDrive::tankDriveVolts,
        robotDrive
    );

    System.out.println("ghjkklghkljk;" + ramseteCommand);

    // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> {
        robotDrive.tankDriveVolts(0, 0);
        System.out.println("hello! you didnt fail this");
      });
  }
}