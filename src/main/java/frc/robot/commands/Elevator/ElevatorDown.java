// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorState;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDown extends Command {
  Elevator elevatorSubsystem;
  boolean isAtSetpoint = false;
  double startingPos;
  /** Creates a new ElevatorDown. */
  public ElevatorDown(Elevator elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPos = elevatorSubsystem.getPosition();
    //RobotContainer.currentElevatorState = ElevatorState.DOWN;
    // if (RobotContainer.currentElevatorState == ElevatorState.L4 || RobotContainer.currentElevatorState == ElevatorState.Net) {
    //   elevatorSubsystem.setPosition(ElevatorHeights.kL3Height);
    // }
    //else {
      elevatorSubsystem.setPosition(0.1);
    //}
    elevatorSubsystem.pidController.setTolerance(0.175);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.currentElevatorState != ElevatorState.DOWN) {
      if (elevatorSubsystem.getPosition() < startingPos/2) {
        RobotContainer.currentElevatorState = ElevatorState.DOWN;
      }
    }
    isAtSetpoint = (Math.abs(elevatorSubsystem.getPosition() - 0.1) < elevatorSubsystem.errorTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (RobotContainer.currentElevatorState != ElevatorState.DOWN) {
      RobotContainer.currentElevatorState = ElevatorState.DOWN;
    }
    elevatorSubsystem.pidController.setTolerance(elevatorSubsystem.errorTolerance);
  }

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
