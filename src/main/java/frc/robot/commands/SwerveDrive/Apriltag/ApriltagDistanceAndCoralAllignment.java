// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.Apriltag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagDistanceAndCoralAllignment extends Command {
  SwerveSubsystem m_swerveDrive;
  boolean runOnce = true;

  SwerveDrive swerveDrive;
  Limelight m_limelight;
  int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double targetTa;
  double targetTx;
  double errorTa;
  double errorTx;
  int targetTimer = 0;

  double minSpeed = 0.05;
  double horizontalSpeed = 1.0;

  PIDController txController = new PIDController(0.065, 0.0, 0.005);
  PIDController taController = new PIDController(0.2, 0, 0.0);
  /** Creates a new ApriltagDistanceAndCoralAllignment. */
  public ApriltagDistanceAndCoralAllignment(SwerveSubsystem swerveSubsystem, Limelight limelight, double tx, double ta, boolean runOnce) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
    this.targetTx = tx;
    this.targetTa = ta;
    this.runOnce = runOnce;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTimer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    errorTa = targetTa - m_limelight.ta;
    errorTx = targetTx - m_limelight.tx;

    // Check if our current Apriltag ID is a valid ID
    isValidID = false;
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
      }
    }

    if(Math.abs(errorTa) < 0.3 && Math.abs(errorTx) < 0.5){
      targetTimer++;
    }

    swerveDrive.drive(new ChassisSpeeds(taController.calculate(m_limelight.ta, targetTa), -1 * txController.calculate(m_limelight.tx, targetTx), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (runOnce){
     if (!isValidID || targetTimer >= 50){
       return true;
     }
  }
    return false;
  }
}
