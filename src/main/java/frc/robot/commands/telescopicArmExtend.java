// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class telescopicArmExtend extends Command {

  double gearboxRatio;
  double motorCMPerCycle;
  double encoder_Value;
  double pAdjustment;
  double error;
  double kP = 0.1;
  double currentDistance;

  /** Creates a new telescopicArmExtend. */
  public telescopicArmExtend(double distanceCM) {

    double gearboxRatio = distanceCM*3/30;
    double motorCMPerCycle = gearboxRatio*15;
    double encoder_Value = motorCMPerCycle*2048;
    this.encoder_Value = encoder_Value;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = RobotContainer.m_telescopicArm.encoderValue();
    error = encoder_Value - currentDistance;
    pAdjustment = encoder_Value*kP;
    RobotContainer.m_telescopicArm.setSpeed(pAdjustment);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
