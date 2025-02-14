// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTillSetTime extends Command {
  double forwardSpeed;
  double horizontalSpeed;
  double rotationalSpeed;
  double time;

  double setTime;

  SwerveDrive swerveDrive;
  /** Creates a new DriveTillSetTime. */
  public DriveTillSetTime(SwerveSubsystem m_SwerveSubsystem, double forwardSpeed, double horizontalSpeed, double rotationalSpeed, double time) {
    this.swerveDrive = m_SwerveSubsystem.swerveDrive;

    this.forwardSpeed = forwardSpeed;
    this.horizontalSpeed = horizontalSpeed;
    this.rotationalSpeed = rotationalSpeed;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setTime = Timer.getFPGATimestamp() + time;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(new ChassisSpeeds(forwardSpeed, horizontalSpeed, rotationalSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() >= setTime){
      return true;
    }
    return false;
  }
}
