// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorSetSpeed extends Command {

Elevator m_elevator;
CommandXboxController m_operatorController;



  public ElevatorSetSpeed(Elevator m_elevator, CommandXboxController operatorController) {
    addRequirements(m_elevator);
    this.m_elevator = m_elevator;
    this.m_operatorController = operatorController;
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double controlOutput = MathUtil.applyDeadband(m_operatorController.getLeftY() * -1, 0.05) * 2;
    double feedforwardOutput = m_elevator.feedforward.calculate(controlOutput);
    m_elevator.setVoltage(controlOutput + feedforwardOutput);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}