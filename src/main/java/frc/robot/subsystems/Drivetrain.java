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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.Robot;

//import frc.robot.Constants.DriveConstants;


public class Drivetrain extends Subsystem {
  // The motors on the left side of the drive.
  private static WPI_TalonFX leftMaster;
  private static WPI_TalonFX leftSlave;
  private static WPI_TalonFX rightMaster;
  private static WPI_TalonFX rightSlave;

  // The robot's drive
  private DifferentialDrive m_drive;

 

  // The gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  // Odometry class for tracking robot pose
  public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  public final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getAngle()));;
  public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.54, 0.153, 0.0234);

  public final PIDController leftPIDController = new PIDController(1.09, 0, 0);
  public final PIDController righPIDController = new PIDController(1.09, 0, 0);
  
  Pose2d pose;

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Pose2d getPose(){
    return pose; 
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftEncoderVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(4.0) / 60,
      getRightEncoderVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(4.0) / 60
    );
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return righPIDController;
  }
 
  public Drivetrain() {
   initTalons();
   leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
   rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
   resetEncoders();
  }

  public void configMagEncoder(WPI_TalonFX talon, boolean phase) {
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    talon.setSensorPhase(phase);
  }

  public void configEncodersForDrive(){
    configMagEncoder(leftMaster, false);
    configMagEncoder(rightMaster, false);
  }

  private void initTalons(){
    rightMaster = new WPI_TalonFX(Constants.d_RightTop);
    rightSlave = new WPI_TalonFX(Constants.d_RightBottom);
    leftMaster = new WPI_TalonFX(Constants.d_LeftTop);
    leftSlave = new WPI_TalonFX(Constants.d_LeftBottom);
    m_drive = new DifferentialDrive(leftMaster, rightMaster);

    //Init Talons
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    
    /*sensorPosition = getEncoders();
    error = setpoint - sensorPosition;
    autOutputSpeed = error * kP;
    */
  }

  public void updatePose(){
    pose = odometry.update(getHeading(), getLeftEncoderPosition(), getRightEncoderPosition());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(-gyro.getAngle())); //potential bug #1
  }
  
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
   }
  
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0) * Constants.kDriveTicks2Feet;
  }

  public double getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0) * Constants.kDriveTicks2Feet;
  }

  public double getRightEncoderVelocity() {
    return leftMaster.getSelectedSensorVelocity(0);
  }

  public double getLeftEncoderVelocity() {
    return rightMaster.getSelectedSensorVelocity(0);
  }


  public void printGyro(){
    SmartDashboard.putNumber("Gyro Angle: ", gyro.getAngle());
  }

  public double getEncodersDistance(){
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  double setpoint = 0;
  final double iLimit = 1;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  public void initPID(){
    Robot.driveTrain.resetEncoders();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();  
  }

  public void driveP(final double kP, final double setpoint){
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double outPutSpeed = kP * error;
    driveSlave(-outPutSpeed, -outPutSpeed);
    System.out.println("Speed: " + outPutSpeed);
  }

  public void drivePLimelight(final double kP, final double setpoint, double steeringAdjust) {
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double outPutSpeed = kP * error;
    //outPutSpeed += steeringAdjust;
    driveSlave(-outPutSpeed - steeringAdjust , -outPutSpeed + steeringAdjust);
  }

  public void drivePI(final double kP, final double kI, double setpoint){
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double outPutSpeed = kP * error + kI * errorSum;
    driveSlave(outPutSpeed, outPutSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
  }

  public void drivePID(final double kP, final double kI, final double kD, double setpoint){
    double sensorPosition = getEncodersDistance();
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

    double outPutSpeed = kP * error + kI * errorSum + kD * errorRate;
    driveSlave(outPutSpeed, outPutSpeed);
  }

  public void driveSlave(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, -left);
    rightMaster.set(ControlMode.PercentOutput, -right);
  }

  public void driveJoystick(){
    driveSlave(Robot.oi.getPs4LeftYaxis(), Robot.oi.getPs4RightYaxis());
  }

  /*public void runFalcons(){
    testFalcon.set(ControlMode.PercentOutput, -1);
  }

  public void testRunFalcons(){
    testFalcon.set(ControlMode.PercentOutput, 1);
  }*/
  @Override
  public void periodic() {
    setDefaultCommand(new DriveCommand());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveCommand());
  }

}

