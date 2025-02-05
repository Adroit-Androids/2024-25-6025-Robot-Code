// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.Apriltag;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToLastSeenApriltagTarget extends Command {
  SwerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  Limelight m_limelight;

  double speed = 0.5;
  /** Creates a new GoToLastSeenApriltagTarget. */
  public GoToLastSeenApriltagTarget(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.currentApriltagID != RobotContainer.currentTargetID){
      swerveDrive.drive(new ChassisSpeeds(0.0, (Math.signum(RobotContainer.lastReadTxTarget) * speed), 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_limelight.currentApriltagID == RobotContainer.currentTargetID){
      return true;
    }
    return false;
  }
}
