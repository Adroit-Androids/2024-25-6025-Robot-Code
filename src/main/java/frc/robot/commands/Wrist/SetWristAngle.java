// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristAngle extends InstantCommand {
  Wrist m_wrist;
  double setAngle;
  public SetWristAngle(Wrist wristSubsytem, double targetDegree) {
    this.m_wrist = wristSubsytem;
    this.setAngle = targetDegree;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.isAtRest = false;
    m_wrist.targetAngle = setAngle;
  }
}
