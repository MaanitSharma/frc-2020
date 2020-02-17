/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
     // on right side so we reverse it if we put the encode on correct. disble if we
    // need to once we test
    public static final int d_RightTop = 2;
    public static final int d_RightBottom = 3;
    public static final int d_LeftTop = 0;
    public static final int d_LeftBottom = 1;

    //Controller Ports
    public static int ps4_port = 0;

    public static int ps4LeftYaxis = 1;
    public static int ps4RightYaxis = 5;
    public static int ps4ShooterButton = 1;
    public static int ps4AutoAdjustButton = 2;
    public static int ps4ColorSensorButton = 3;
    public static int ps4IntakeOuttakeButton = 8;
    public static int ps4IntakeIntakeButton = 7;
    public static int ps4IntakeDeployButton = 5;

    public static final boolean LEFT_ENCODER_REVERSED = false;

    public static final boolean RIGHT_ENCODER_REVERSED = true;

    public static final double kTrackwidthMeters = 0.27289392026922016;

    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveTicks2Feet = 1.0 / 4096 * 4 * Math.PI / 12;
    public static final double kDeg2Talon4096Unit = 1 / 360.0 * 4096.0;
    public static final double kTalon4096Unit2Deg = 1 / kDeg2Talon4096Unit;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.

    public static final double ksVolts = 0.54;

    public static final double kvVoltSecondsPerMeter = 0.153;

    public static final double kaVoltSecondsSquaredPerMeter = 0.0234;

    // Example value only - as above, this must be tuned for your drive!
    //optimal controller gains used here
    public static final double kPDriveVel = 1.09; 
    //9.31;

    public static final double kDDriveVel = 0;

	public static final double ENCODER_CPR = 4096.0;



    public static final int kDriverControllerPort = 0;


    public static final double kMaxSpeedMetersPerSecond =0.01;// 0.25;

    public static final double kMaxAccelerationMetersPerSecondSquared = 0.01;//0.25;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    //dont change these values
    public static final double kRamseteB = 2;

    public static final double kRamseteZeta = 0.7;

}