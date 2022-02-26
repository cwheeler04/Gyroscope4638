// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DriveConstants {
    public static final int kLeftMotor1Port = 10;
    public static final int kLeftMotor2Port = 11;
    public static final int kRightMotor1Port = 8;
    public static final int kRightMotor2Port = 9;
    
    public static final double kTicksPerMeter = 14.62; //(1/3544.85)*0.02.5
    public static final double kEncoderDistancePerPulse = 1.0 / kTicksPerMeter; //((1/20)*0.1524*Math.PI)/4096.0
    //public static final double kEncoderDistancePerPulse = 20;
    
    public static final int[] kLeftEncoderPorts = new int[] {kLeftMotor1Port, kLeftMotor2Port};
    public static final int[] kRightEncoderPorts = new int[] {kRightMotor1Port, kRightMotor2Port};
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;

    public static final double ksVolts = 0.12763;
  public static final double kvVoltSecondsPerMeter = 0.26026;
  public static final double kaVoltSecondsSquaredPerMeter = 0.071788;
  public static final double kPDriveVel = 2.3672;
 // public static final double kTrackwidthMeters = 0.555; //TODO: add our width, was 0.555
  public static final double kTrackwidthMeters = 0.555;
  public static final double kMaxSpeedMetersPerSecond = 1.25;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
  public static final double kRamseteB = 1.0;
  public static final double kRamseteZeta = 0.4;
  public static final DifferentialDriveKinematics kDriveKinematics =
  new DifferentialDriveKinematics(kTrackwidthMeters);

}
