// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDrive.DriveTillSetTime;
import frc.robot.commands.SwerveDrive.TargetPoseAllignment;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefAllignment extends SequentialCommandGroup {
  /** Creates a new reefAllignmentRight. */
  public ReefAllignment(SwerveSubsystem swerveSubsystem, Limelight limelight, double leftDistance, double forwardDistance, double driveVelocity, double driveTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TargetPoseAllignment(swerveSubsystem, limelight, leftDistance, forwardDistance), new DriveTillSetTime(swerveSubsystem, driveVelocity, 0.0, 0.0, driveTime));

  }
}
