// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.nio.file.Path;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /*
  private AHRS ahrs;
  */
  private Command m_autonomousCommand;
  

  private RobotContainer m_robotContainer;

  //String trajectoryJSON = "paths/Path1.wpilib.json";
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   
    
    /*
    ahrs = new AHRS(SerialPort.Port.kUSB); 
    SmartDashboard.putString("Calibrated", "currently calibrating");
    ahrs.calibrate();
    SmartDashboard.putString("Calibrated", "Calibration complete");
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    */
    m_robotContainer = new RobotContainer();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
    SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());
    SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());
    SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());
    SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
    SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
    SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
    SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
    SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
    SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
    SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
    SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
    SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
    SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
    SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
    SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
    SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
    SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
    //AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    //SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    //SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
    SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
    SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
    SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
    SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
    SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
    SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
    SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
    SmartDashboard.putNumber("Rotation", ahrs.getRotation2d().getDegrees());
    */
    
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
