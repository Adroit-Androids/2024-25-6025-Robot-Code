// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.swerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class absoluteDrive extends Command {
  public swerveSubsystem m_swerveSubsystem;
  CommandXboxController robotController;

  /** Creates a new absoluteDrive. */
  public absoluteDrive(swerveSubsystem m_swerveSubsystem, CommandXboxController m_robotController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.robotController = m_robotController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.arcadeDrive(RobotContainer.m_driverController.getRightX(),
                                       robotController.getRightY(),
                                       robotController.getLeftX(),
                                       robotController.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
