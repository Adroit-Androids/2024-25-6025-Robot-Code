// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorSetSpeed extends Command {

Elevator m_elevator;
CommandXboxController m_driverController;



  public ElevatorSetSpeed(Elevator m_elevator, CommandXboxController m_driveController) {
    addRequirements(m_elevator);
    this.m_elevator = m_elevator;
    this.m_driverController = m_driveController;
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevator.movetoVelocity(m_driverController.getLeftY() * Constants.ElevatorTrapezoid.maxVelocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}