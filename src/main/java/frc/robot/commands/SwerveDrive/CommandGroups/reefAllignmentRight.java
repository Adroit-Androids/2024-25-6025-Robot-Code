// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDrive.apriltagAllignment;
import frc.robot.commands.SwerveDrive.apriltagDistance;
import frc.robot.commands.SwerveDrive.coralAllignment;
import frc.robot.subsystems.Limelight.limelight;
import frc.robot.subsystems.Swerve.swerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class reefAllignmentRight extends SequentialCommandGroup {
  /** Creates a new reefAllignmentRight. */
  public reefAllignmentRight(swerveSubsystem swerveSubsystem, limelight limelight, double targetTx, double targetTa) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new apriltagAllignment(swerveSubsystem, limelight), new apriltagDistance(swerveSubsystem, limelight, targetTa), new coralAllignment(swerveSubsystem, limelight, targetTx));
  }
}
