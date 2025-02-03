// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.limelight;
import frc.robot.subsystems.Swerve.swerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class apriltagDistance extends Command {
  swerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  limelight m_limelight;
  int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double targetTa;
  double error;
  int targetTimer = 0;

  double minSpeed = 0.1;

  PIDController taController;

  /** Creates a new coralApriltagDistance. 
   * @param swerveDrive Swerve drive subsystem for the command to run on 
   * @param limelight limelight subsystem to get the tx and ta values
   * @param ta Requested ta value to be set to
  */
  public apriltagDistance(swerveSubsystem swerveSubsystem, limelight limelight, double ta) {
    taController = new PIDController(0.15, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
    this.targetTa = ta;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetTa - m_limelight.ta;

    // Check if our current Apriltag ID is a valid ID
    isValidID = false;
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
      }
    }

    // Increase targetTimer if our absolute error is less than a certain value
    if (Math.abs(error) < 0.5){
      targetTimer++;
    }
    else{
      targetTimer = 0;
    }

    swerveDrive.drive(new ChassisSpeeds(taController.calculate(m_limelight.ta, targetTa) + (minSpeed * Math.signum(error)),
                        0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End command if our Apriltag ID is not a valid ID or if our target timer has reached a certain value
    if (targetTimer >= 50 || !isValidID || m_limelight.ta == 0){
      return true;
     }
    else{
      return false;
    }

  }
}
