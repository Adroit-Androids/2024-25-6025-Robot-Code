// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.limelight;
import frc.robot.subsystems.Swerve.swerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralAllignment extends Command {
  swerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  limelight m_limelight;
  double[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double targetTx;

  PIDController txController = new PIDController(0.005, 0, 0);


  /** Creates a new coralAllignment. 
   * @param swerveDrive Swerve drive subsystem for the command to run on 
   * @param limelight limelight subsystem to get the tx and ta values
   * @param tx Requested tx value to be set to
  */
  public coralAllignment(swerveSubsystem swerveSubsystem, limelight limelight, double tx) {
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
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
      }
    }
  }

  public ChassisSpeeds getTargetChassisSpeedsTx(double speed){
    double angle = m_swerveDrive.swerveDrive.getOdometryHeading().getDegrees() + 90;
    Translation2d translativeValues = new Translation2d(speed, new Rotation2d(Math.toRadians(angle)));

    ChassisSpeeds chassisSpeeds = swerveDrive.swerveController.getRawTargetSpeeds(translativeValues.getX(), 
                                                                            translativeValues.getY(), 
                                                                            swerveDrive.getOdometryHeading().getRadians(), 
                                                                            swerveDrive.getOdometryHeading().getRadians());

    return chassisSpeeds;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.setChassisSpeeds(getTargetChassisSpeedsTx(txController.calculate(m_limelight.tx, targetTx)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End command if our Apriltag ID is not a valid ID 
    if (targetTx -0.5 <= m_limelight.tx & m_limelight.tx <= targetTx +0.5){
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
