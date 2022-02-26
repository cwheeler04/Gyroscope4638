// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public static final double ksVolts = 0.12763;
  public static final double kvVoltSecondsPerMeter = 0.26026;
  public static final double kaVoltSecondsSquaredPerMeter = 0.071788;
  public static final double kPDriveVel = 2.3672;
  public static final double kTrackwidthMeters = 0.69; //TODO: add our width 
  public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  

  private CANSparkMax m_leftFront = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
  private RelativeEncoder m_rightFrontEncoder;
  private RelativeEncoder m_rightBackEncoder;
  private RelativeEncoder m_leftFrontEncoder;
  private RelativeEncoder m_leftBackEncoder;
  


  private MotorControllerGroup m_left;
  private MotorControllerGroup m_right;
  private DifferentialDrive m_robotDrive;

  /**private MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(
      new PWMSparkMax(DriveConstants.kLeftMotor1Port),
      new PWMSparkMax(DriveConstants.kLeftMotor2Port));

// The motors on the right side of the drive.
private MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(
      new PWMSparkMax(DriveConstants.kRightMotor1Port),
      new PWMSparkMax(DriveConstants.kRightMotor2Port));
// The robot's drive
private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
**/
// The left-side drive encoder
private final Encoder m_leftEncoder =
  new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);

// The right-side drive encoder
private final Encoder m_rightEncoder =
  new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);

// The gyro sensor
private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

// Odometry class for tracking robot pose
private final DifferentialDriveOdometry m_odometry;
  public DriveSubsystem() {
    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_leftBack.setIdleMode(IdleMode.kBrake);
    m_rightBack.setIdleMode(IdleMode.kBrake);
    m_leftFront.setInverted(true);
    m_leftBack.setInverted(true);
    SmartDashboard.putBoolean("gyro connection", m_gyro.isConnected());
    m_left  = new MotorControllerGroup(m_leftFront, m_leftBack);
    m_right = new MotorControllerGroup(m_rightFront, m_rightBack);
    m_robotDrive = new DifferentialDrive(m_left, m_right);
    m_robotDrive.setDeadband(0.15);
    m_robotDrive.setSafetyEnabled(false);
    m_rightFrontEncoder = m_rightFront.getEncoder();
    m_rightBackEncoder = m_rightBack.getEncoder();
    m_leftFrontEncoder = m_leftFront.getEncoder();
    m_leftBackEncoder = m_leftBack.getEncoder();
    m_leftBackEncoder.setPosition(0);
    m_leftFrontEncoder.setPosition(0);
    m_rightBackEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);

    /*m_rightFrontEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightBackEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightBackEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftFrontEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftBackEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftBackEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);*/
  
  
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
   // m_left.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    Rotation2d r = m_gyro.getRotation2d();
    m_odometry = new DifferentialDriveOdometry(r);
    System.out.printf("Inital heading equals %f", r.getRadians());

  }

  @Override
  public void periodic() {
    //System.out.println("left side velocity " + getLeftEncoderSpeed() + " right side velocity " + getRightEncoderSpeed());
    SmartDashboard.putNumber("right velocity", getRightEncoderSpeed());
    SmartDashboard.putNumber("left velocity", getLeftEncoderSpeed());
     // Update the odometry in the periodic block
     m_odometry.update(
      m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
      
    SmartDashboard.putNumber("right encoder distance", getRightEncoderDistance());
    SmartDashboard.putNumber("left encoder distance", getLeftEncoderDistance());
    
    SmartDashboard.putNumber("Odometry X: ", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y: ", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Degrees: ", m_odometry.getPoseMeters().getRotation().getDegrees());
  }  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_robotDrive.arcadeDrive(fwd, rot);
  }
   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //m_leftMotors.setVoltage(leftVolts);
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    m_robotDrive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_leftBackEncoder.setPosition(0);
    m_leftFrontEncoder.setPosition(0);
    m_rightBackEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  private Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  private Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_robotDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  public double getRightEncoderDistance(){
    return (m_rightFrontEncoder.getPosition())*DriveConstants.kEncoderDistancePerPulse;
    
  }
  public double getLeftEncoderDistance(){
    return (m_leftFrontEncoder.getPosition())*DriveConstants.kEncoderDistancePerPulse; 
    
  }
  public double getLeftEncoderSpeed(){
    return ((m_leftFrontEncoder.getVelocity())*DriveConstants.kEncoderDistancePerPulse)/60;
  }
  
  public double getRightEncoderSpeed(){
    return ((m_rightFrontEncoder.getVelocity())*DriveConstants.kEncoderDistancePerPulse)/60;
  }
}
