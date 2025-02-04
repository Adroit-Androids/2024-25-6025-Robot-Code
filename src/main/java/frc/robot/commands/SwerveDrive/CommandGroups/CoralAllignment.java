// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.CommandGroups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralAllignment extends Command {
  SwerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  Limelight m_limelight;
  double[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double targetTx;
  double error;

  double minSpeed = 0.1;

  //Value wich increases everytime our current tx is in a certain range of our target tx
  //Made to prevent the command ending when it overshoots
  int targetTimer;

  PIDController txController = new PIDController(0.02, 0.0, 0);


  /** Creates a new coralAllignment. 
   * @param swerveDrive Swerve drive subsystem for the command to run on 
   * @param limelight limelight subsystem to get the tx and ta values
   * @param tx Requested tx value to be set to
  */
  public CoralAllignment(SwerveSubsystem swerveSubsystem, Limelight limelight, double tx) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
    this.targetTx = tx;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTimer = 0;
  }

  public ChassisSpeeds getTargetChassisSpeedsTx(double speed){

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, -speed, 0.0);

    return chassisSpeeds;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetTx - m_limelight.tx;

    isValidID = false;
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
      }
    }

    if (Math.abs(error) < 0.5){
      targetTimer++;
     }
    else{
      targetTimer = 0;
    }

    swerveDrive.drive(new ChassisSpeeds(0, 
                                        -1 * (txController.calculate(m_limelight.tx, targetTx) + (minSpeed * Math.signum(minSpeed))),
                                        0));
     SmartDashboard.putNumber("Target time", targetTimer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End command if our Apriltag ID is not a valid ID 
    if (targetTimer >= 25 || !isValidID){
      return true;
     }
    else{
      if (!isValidID){
        return true;
      }
      return false;
    }
  }
}
