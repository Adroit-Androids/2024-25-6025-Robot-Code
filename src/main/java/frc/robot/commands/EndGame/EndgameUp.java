// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndGame;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Endgame.Endgame;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndgameUp extends Command {
  Endgame m_endgame;

  /** Creates a new EndgameUp. */
  public EndgameUp(Endgame endgame) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endgame);

    this.m_endgame = endgame;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endgame.setSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endgame.setSpeedZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
