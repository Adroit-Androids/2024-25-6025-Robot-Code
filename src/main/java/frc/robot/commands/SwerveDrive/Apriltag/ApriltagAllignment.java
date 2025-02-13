// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.Apriltag;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagAllignment extends Command {
  SwerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  Limelight m_limelight;
  int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  int targetTimer = 0;
  double targetAngle;
  double error;
  double currentAngle;
  double minSpeed = 0.0;
  double pAdjustment = 0.0;

  double kP = 5.0;

  PIDController angularVelocityController;

  /** Creates a new apriltagAllignment. */
  public ApriltagAllignment(SwerveSubsystem swerveSubsystem, Limelight limelight) {
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
        RobotContainer.currentTargetID = m_limelight.currentApriltagID;
      }
    }

    if (isValidID){

      if (m_limelight.currentApriltagID == 21 || m_limelight.currentApriltagID == 7){
        targetAngle = 0;
      }
      if (m_limelight.currentApriltagID == 22 || m_limelight.currentApriltagID == 6){
        targetAngle = 60;
      }
      if (m_limelight.currentApriltagID == 17 || m_limelight.currentApriltagID == 11){
        targetAngle = 120;
      }
      if (m_limelight.currentApriltagID == 18 || m_limelight.currentApriltagID == 10){
        targetAngle = 180;
      }
      if (m_limelight.currentApriltagID == 19 || m_limelight.currentApriltagID == 9){
        targetAngle = 240;
      }
      if (m_limelight.currentApriltagID == 20 || m_limelight.currentApriltagID == 8){
        targetAngle = 300;
      }

    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerveDrive.getOdometryHeading().getDegrees() + 180;
    error = targetAngle - currentAngle;
    if (error < -180){
      error += 360;
    }
    pAdjustment = error * kP;
    
    // Increase targetTimer if we are within a certain range of our target angle
    if (Math.abs(error) <= 1.0){
      ++targetTimer;
    }
    else{
      targetTimer = 0;
    }

    if (m_limelight.currentApriltagID == RobotContainer.currentTargetID){
      RobotContainer.lastReadTxTarget = m_limelight.tx;
    }
    
    swerveDrive.drive(new Translation2d(),
                      Math.toRadians(pAdjustment),
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
    if (targetTimer >= 5 || Math.abs(error) >= 50 || !isValidID){
      return true;
     }
    else{
      return false;
    }
    
  }
}
