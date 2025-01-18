// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.limelight;
import frc.robot.subsystems.Swerve.swerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class apriltagAllignment extends Command {
  public swerveSubsystem m_swerveDrive;
  limelight m_limelight;
  double[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double targetAngle;

  /** Creates a new apriltagAllignment. */
  public apriltagAllignment(swerveSubsystem swerveDrive, limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.,
    addRequirements(swerveDrive);
    this.m_swerveDrive = swerveDrive;
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

      if (m_limelight.currentApriltagID == 6 || m_limelight.currentApriltagID == 22){
        targetAngle = 120;
      }

      if (m_limelight.currentApriltagID == 17 || m_limelight.currentApriltagID == 11){
        targetAngle = 60;
      }

      if (m_limelight.currentApriltagID == 19 || m_limelight.currentApriltagID == 9){
        targetAngle = -60;
      }

      if (m_limelight.currentApriltagID == 20 || m_limelight.currentApriltagID == 8){
        targetAngle = -120;
      }
    }
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.swerveDrive.drive(null, targetAngle, true, false);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End command if our Apriltag ID is not a valid ID 
    if (isValidID){
      return false;
    }
    if (targetAngle == m_swerveDrive.robotRotation){
      return true;
     }
    else{
      return true;
    }

  }
}
