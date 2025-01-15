// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TelescopicArmCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.telescopicArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class telescopicArmExtend extends Command {
  ProfiledPIDController pidController;
  telescopicArm m_telescopicArm;

  double gearboxRatio;
  double motorCMPerCycle;
  double encoder_Value;
  double pAdjustment;
  double currentDistance;

  /** Creates a new telescopicArmExtend. */
  public telescopicArmExtend(double distanceCM, telescopicArm m_telescopicArm) {
    this.m_telescopicArm = m_telescopicArm;
    this.pidController = m_telescopicArm.armController;

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
    currentDistance = m_telescopicArm.encoderValue();
    pAdjustment = pidController.calculate(currentDistance, encoder_Value);
    m_telescopicArm.setSpeed(pAdjustment);
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
