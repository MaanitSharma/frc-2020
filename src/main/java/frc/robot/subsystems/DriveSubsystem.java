/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

  public Pose2d getPose(){
    return Robot.driveTrain.getPose(); 
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return Robot.driveTrain.getWheelSpeeds();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    System.out.println("OUTPUT LEFT VOL: " + leftVolts);
    System.out.println("OUTPUT RIGHT VOL: " + rightVolts);
    Robot.driveTrain.tankDriveVolts(leftVolts, rightVolts);
  }

  public SimpleMotorFeedforward getFeedForward(){
    return Robot.driveTrain.getFeedForward();
  }

  public DifferentialDriveKinematics getKinematics(){
    return Robot.driveTrain.getKinematics();
  }

  public PIDController getLeftPIDController(){
    return Robot.driveTrain.getLeftPIDController();
  }

  public PIDController getRightPIDController(){
    return Robot.driveTrain.getRightPIDController();
  }
 
  public DriveSubsystem() {

  }

  public void justAFK()
  {
    System.out.println("afakfakfakfafk");
  }

  public void resetOdometry(Pose2d pose)
  {
    Robot.driveTrain.resetOdometry(pose);
  }

  @Override
  public void periodic(){
    Robot.driveTrain.updatePose();
  }

}

