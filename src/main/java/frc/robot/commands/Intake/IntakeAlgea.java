// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgea extends Command {
  Intake m_intake;
  Wrist m_wrist;
  double newAngle;
  /** Creates a new IntakeAlgea. */
  public IntakeAlgea(Intake intakeSubystem, Wrist wristSubsytem, double setAngle) {
    m_intake = intakeSubystem;
    m_wrist = wristSubsytem;
    newAngle = setAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.targetAngle = newAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeVoltage(-0.57);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeVoltage(-0.25);
    m_wrist.targetAngle = 90;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
