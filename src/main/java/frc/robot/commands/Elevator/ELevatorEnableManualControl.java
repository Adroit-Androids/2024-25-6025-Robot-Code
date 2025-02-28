// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorState;
import frc.robot.subsystems.Elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ELevatorEnableManualControl extends InstantCommand {
  Elevator m_elevator;
  public ELevatorEnableManualControl(Elevator elevatorSubsytem) {
    m_elevator = elevatorSubsytem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.isManualControl) {
      m_elevator.isManualControl = false;
      m_elevator.pidController.reset(m_elevator.getPosition(), 0);
      m_elevator.pidController.setGoal(0);
      RobotContainer.currentElevatorState = ElevatorState.DOWN;
    }
    else {
      m_elevator.isManualControl = true;
    }
  }
}
