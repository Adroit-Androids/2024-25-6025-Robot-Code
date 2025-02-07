// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

public class Limelight extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private SwerveSubsystem swerveSubsystem;
  private String ll_table = "limelight";
  public double tx;
  public double ty;
  public double ta;
  public double currentApriltagID;
  public LimelightHelpers.PoseEstimate limelightPoseEstimate;
  public boolean doRejectUpdate = false; 

  /** Creates a new limelight. */
  public Limelight(SwerveSubsystem m_swerveDrive) {
    this.swerveDrive = m_swerveDrive.swerveDrive;
    this.swerveSubsystem = m_swerveDrive;
    LimelightHelpers.setCameraPose_RobotSpace(ll_table, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    limelightPoseEstimate = new LimelightHelpers.PoseEstimate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LimelightHelpers.SetRobotOrientation(ll_table, swerveDrive.getYaw().getDegrees(), Math.toDegrees(swerveDrive.getRobotVelocity().omegaRadiansPerSecond),
                                          swerveDrive.getPitch().getDegrees(), 0.0, 0.0, 0.0);


    doRejectUpdate = false;
    if(Math.abs(Math.toDegrees(swerveDrive.getRobotVelocity().omegaRadiansPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(limelightPoseEstimate.tagCount == 0)
      {
        doRejectUpdate = true;
      }
    
    tx = LimelightHelpers.getTX(ll_table);
    ty = LimelightHelpers.getTY(ll_table);
    ta = LimelightHelpers.getTA(ll_table);
    currentApriltagID = LimelightHelpers.getFiducialID(ll_table);
    

    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
    SmartDashboard.putNumber("Apriltag ID", currentApriltagID);
  }
}
