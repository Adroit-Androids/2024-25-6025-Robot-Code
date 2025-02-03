// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.limelight;
import frc.robot.subsystems.Swerve.swerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class apriltagAllignment extends Command {
  swerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  limelight m_limelight;
  int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double targetAngle;
  Rotation2d targeRotation2d;
  int targetTimer = 0;
  double error;
  double currentAngle;
  double minSpeed = 1.0;

  PIDController angularVelocityController;

  /** Creates a new apriltagAllignment. */
  public apriltagAllignment(swerveSubsystem swerveSubsystem, limelight limelight) {
    angularVelocityController = new PIDController(3.5, 3.5, 0.0);
    // Use addRequirements() here to declare subsystem dependencies.,
    addRequirements(swerveSubsystem);
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
      }
    }

    if (isValidID){

      if (m_limelight.currentApriltagID == 22 || m_limelight.currentApriltagID == 8){
        targetAngle = 60;
      }
      if (m_limelight.currentApriltagID == 21 || m_limelight.currentApriltagID == 7){
        targetAngle = 0;
      }
      if (m_limelight.currentApriltagID == 20 || m_limelight.currentApriltagID == 8){
        targetAngle = -60;
      }
      if (m_limelight.currentApriltagID == 19 || m_limelight.currentApriltagID == 9){
        targetAngle = -120;
      }
      if (m_limelight.currentApriltagID == 18 || m_limelight.currentApriltagID == 10){
        targetAngle = -180;
      }
      if (m_limelight.currentApriltagID == 17 || m_limelight.currentApriltagID == 11){
        targetAngle = 120;
      }

    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerveDrive.getOdometryHeading().getDegrees();
    error = targetAngle - currentAngle;
    
    // Increase targetTimer if we are within a certain range of our target angle
    if (Math.abs(error) <= 0.5){
      ++targetTimer;
    }
    else{
      targetTimer = 0;
    }
    
    swerveDrive.drive(new Translation2d(),
                      Math.toRadians(angularVelocityController.calculate(currentAngle, error) + Math.toRadians(minSpeed * Math.signum(error))),
                      true, false);

    SmartDashboard.putBoolean("Command is finished", isFinished());
    SmartDashboard.putNumber("Target angle", targetAngle);
    SmartDashboard.putNumber("Current angle", swerveDrive.getOdometryHeading().getDegrees());
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End command if our Apriltag ID is not a valid ID 
    if (targetTimer >= 50 || Math.abs(error) >= 50 || !isValidID){
      return true;
     }
    else{
      return false;
    }
    
  }
}
