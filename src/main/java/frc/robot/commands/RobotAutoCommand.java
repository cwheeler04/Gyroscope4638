// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotAutoCommand extends SequentialCommandGroup {
  /** Creates a new RobotAutoCommand. */
  public RobotAutoCommand(DriveSubsystem driveSubsystem, Trajectory dTraj_1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveAutoCommand(driveSubsystem, dTraj_1).getRamseteCommand(),
    new DriveAutoCommand(driveSubsystem, dTraj_1).getRamseteCommand()
    );
  }
}
