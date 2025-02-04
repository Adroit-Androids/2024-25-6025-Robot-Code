// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private String ll_table = "limelight";
  public double tx;
  public double ty;
  public double ta;
  public double currentApriltagID;

  //TODO: Add limelight pose estimation
  LimelightHelpers.PoseEstimate limelightMeasurement;

  /** Creates a new limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
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
