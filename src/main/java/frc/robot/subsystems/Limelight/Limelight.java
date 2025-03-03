// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class Limelight extends SubsystemBase {
  private SwerveSubsystem swerveDrive;
  private String ll_table = "limelight";
  public double tx;
  public double ty;
  public double ta;
  public double currentApriltagID;
  public LimelightResults results;
  public LimelightHelpers.PoseEstimate limelightPoseEstimate;
  public boolean doRejectUpdate = false; 
  StructPublisher<Pose2d> limelightPosePublisher;
  StructPublisher<Pose3d> apriltagPoseToRobotPublisher;

  /** Creates a new limelight. */
  public Limelight(SwerveSubsystem m_swerveDrive) {
    LimelightHelpers.setCameraPose_RobotSpace(ll_table, 0.325, 0.09, 0.18, 0.0, 15.0, 0.0);
    limelightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/Limelight Pose", Pose2d.struct).publish();
    apriltagPoseToRobotPublisher = NetworkTableInstance.getDefault().getStructTopic("/Apriltag Pose", Pose3d.struct).publish();
    this.swerveDrive = m_swerveDrive;
    limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll_table);
  }

  public double[] getTargetPose2d() {
    //return LimelightHelpers.getTargetPose_RobotSpace(ll_table);
    return LimelightHelpers.getBotPose_TargetSpace(ll_table);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll_table);

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
    
    limelightPosePublisher.set(limelightPoseEstimate.pose);
    SmartDashboard.putNumberArray("Target pose robot space", getTargetPose2d());
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
    SmartDashboard.putNumber("Apriltag ID", currentApriltagID);
  }
}
