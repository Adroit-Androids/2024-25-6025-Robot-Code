// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAlgea1 extends Command {
  private Elevator m_elevator;
  boolean isAtSetpoint = false;
  /** Creates a new ElevatorL2. */
  public ElevatorAlgea1(Elevator elevatorSubsystem) {
    this.m_elevator = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPosition(ElevatorHeights.firstAlgea);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isAtSetpoint = (Math.abs(m_elevator.getPosition() - Constants.ElevatorHeights.firstAlgea) < m_elevator.errorTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAtSetpoint){
      return true;
    }
    else {
      return false;  
    }
  }
}
