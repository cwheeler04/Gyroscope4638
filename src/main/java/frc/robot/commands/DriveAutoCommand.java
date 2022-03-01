// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAutoCommand extends CommandBase {
  /** Creates a new DriveAutoCommand. */
  private Trajectory m_trajectory;
  private DriveSubsystem m_driveSubsystem;

  public DriveAutoCommand(DriveSubsystem dSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dSubsystem);
    m_driveSubsystem = dSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String trajectoryJSON = "FullCircle3.wpilib.json";
    Trajectory m_trajectory = new Trajectory();
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
   /* Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path

            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

            List.of(new Translation2d(3, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4, -4, new Rotation2d(Math.PI/2)),
            // Pass config 
            config);
    */
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            m_trajectory,
            m_driveSubsystem::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
            m_driveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_driveSubsystem::tankDriveVolts,
            m_driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.zeroHeading();
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
    

    // Run path following command, then stop at the end.
    ramseteCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
