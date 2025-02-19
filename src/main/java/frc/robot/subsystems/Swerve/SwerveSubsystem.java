// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.imu.NavXSwerve;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  

  /**
   * Swerve drive object.
   */
  public final SwerveDrive swerveDrive;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public        double      maximumSpeed = 1.5;
  /**
   * Maximum rotational speed of the robot in radians per second, used to limit acceleration.
   */
  public        double      maximumRotationSpeed = Math.toRadians(135);
  /**
   * Robot configuration gathered from pathplanner
   */
  public RobotConfig robotConfig;
  /**
   * Enable vision odometry updates while driving.
   */
  public final boolean             visionDriveTest     =  false;
  
  /**
   * Innitialive {@link SwerveDrive} with the directory provided
   * 
   * @param directory Directory of the swerve drive json file
   * 
   */
  public SwerveSubsystem(File directory) {
    // Configure how much telemetry  data is sent
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      swerveDrive.swerveDriveConfiguration.imu = new NavXSwerve(NavXComType.kMXP_SPI);
    }catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    if (Robot.isSimulation()){
      resetOdometry(new Pose2d(5, 7.5, new Rotation2d()));
      swerveDrive.setHeadingCorrection(false);
      swerveDrive.setCosineCompensator(false);
    }

    if (visionDriveTest){
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(7, 7, Math.toRadians(2)));
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    setUpPathplanner();

  }


  public void setUpPathplanner(){
   
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }


  //Function to return the pose of the robot
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

    //Function to reset the odometry pose
  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  //Function to get robot velocity
  public ChassisSpeeds getRobotVelocity(){
    return swerveDrive.getRobotVelocity();
  }

  //Function set robot chassisSpeeds
  public void setChassisSpeeds(ChassisSpeeds speeds){
    swerveDrive.drive(speeds);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // When vision is enabled we must manually update odometry in SwerveDrive
    LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.getYaw().getDegrees(), Math.toDegrees(swerveDrive.getRobotVelocity().omegaRadiansPerSecond),
                                           swerveDrive.getPitch().getDegrees(), 0.0, 0.0, 0.0);

    switch (RobotContainer.currentElevatorState){
      case DOWN:
        swerveDrive.setMaximumAllowableSpeeds(maximumSpeed * 1.0, maximumRotationSpeed * 1.0);
        break;
      case PROCESSOR:
        swerveDrive.setMaximumAllowableSpeeds(maximumSpeed * 0.9, maximumSpeed * 0.9);
        break;
      case L0:
        swerveDrive.setMaximumAllowableSpeeds(maximumSpeed * 0.9, maximumSpeed * 0.9);
        break;
      case L1:
        swerveDrive.setMaximumAllowableSpeeds(maximumSpeed * 0.8, maximumSpeed * 0.8);
        break;
      case L2:
        swerveDrive.setMaximumAllowableSpeeds(maximumSpeed * 0.7, maximumSpeed * 0.7);
        break;
      case L3:
        swerveDrive.setMaximumAllowableSpeeds(maximumSpeed * 0.6, maximumSpeed * 0.6);
        break;
    }

    SmartDashboard.putNumber("Robot absolute degree", swerveDrive.getOdometryHeading().getDegrees() + 180);
    if (getCurrentCommand() != null) {
      SmartDashboard.putString("Current swerve command", getCurrentCommand().getName());
    }

    if (getDefaultCommand() != null){
      SmartDashboard.putString("Default swerve command", getDefaultCommand().getName());
    }
    if (visionDriveTest)
     {
    //   if (!RobotContainer.m_limelight.doRejectUpdate){
    //     swerveDrive.addVisionMeasurement(RobotContainer.m_limelight.limelightPoseEstimate.pose, RobotContainer.m_limelight.limelightPoseEstimate.timestampSeconds);
    //   }
      swerveDrive.updateOdometry();
    }
  }
}
