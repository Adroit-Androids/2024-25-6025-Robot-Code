// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoralSetTime extends Command {
  Intake m_intake;
  
  double setTime;
  double time;  
  double appliedPercentageOutput; 

  /** Creates a new ShootAlgeaSetTime. */
  public ShootCoralSetTime(Intake intakeSubsytem, double commandTime, double percentageOutput) {
    m_intake = intakeSubsytem;
    time = commandTime;
    appliedPercentageOutput = percentageOutput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setTime = Timer.getFPGATimestamp() + time;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeVoltage(-appliedPercentageOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > setTime) {
      return true;
    }
    else {
      return false;
    }
  }
}
