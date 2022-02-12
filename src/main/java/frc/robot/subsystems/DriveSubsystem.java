// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public static final double ksVolts = 0.11358;
  public static final double kvVoltSecondsPerMeter = 0.26346;
  public static final double kaVoltSecondsSquaredPerMeter = 0.066365;
  public static final double kPDriveVel = 0.37038;
  public static final double kTrackwidthMeters = 0.69; //TODO: add our width 
  public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;



  private MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(
      new PWMSparkMax(DriveConstants.kLeftMotor1Port),
      new PWMSparkMax(DriveConstants.kLeftMotor2Port));

// The motors on the right side of the drive.
private MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(
      new PWMSparkMax(DriveConstants.kRightMotor1Port),
      new PWMSparkMax(DriveConstants.kRightMotor2Port));
// The robot's drive
private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

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
private final AHRS m_gyro = new AHRS();

// Odometry class for tracking robot pose
private final DifferentialDriveOdometry m_odometry;
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
