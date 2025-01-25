// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.vision;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class swerveSubsystem extends SubsystemBase {
  

  /**
   * Swerve drive object.
   */
  public final SwerveDrive swerveDrive;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public        double      maximumSpeed = 12;
  /**
   * Robot configuration gathered from pathplanner
   */
  public RobotConfig robotConfig;
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean             visionDriveTest     = true;
  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private vision vision;
  
  
  /**
   * Innitialive {@link SwerveDrive} with the directory provided
   * 
   * @param directory Directory of the swerve drive json file
   */
  public swerveSubsystem(File directory) {
    // Configure how much telemetry  data is sent
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    }catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);

    if (visionDriveTest){
      setupPhotonVision();
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
                    new PIDConstants(0.0005, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision(){
    vision = new vision(swerveDrive::getPose, swerveDrive.field);
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
    swerveDrive.driveFieldOriented(speeds);
    
  }
  
  /**
   * Command to drive the robot using translative values and heading as a setpoint
   * 
   * @param translationX translation in x direction
   * @param translationY translation in y direction
   * @param headingX Heading as X to calculate angle of the joysticks
   * @param headingY Heading as y to calculate angle of the joysticks
   */
  public void arcadeDrive(double translationX, Double translationY, Double headingY, Double headingX){
    
    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d( MathUtil.applyDeadband(translationY, OperatorConstants.kLeftJoystickDeadband),
                                                                                MathUtil.applyDeadband(translationX, OperatorConstants.kLeftJoystickDeadband)),
                                                                                maximumSpeed);
    ChassisSpeeds chassisSpeeds = swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                 headingY, headingX,
                                                 swerveDrive.getOdometryHeading().getRadians(), maximumSpeed);

    swerveDrive.driveFieldOriented(chassisSpeeds);
  }

  public void robotRelativeDrive(double translationX, double translationY, double angularVelocity){
    
    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(MathUtil.applyDeadband(translationY, OperatorConstants.kLeftJoystickDeadband),
                                                                                MathUtil.applyDeadband(translationX, OperatorConstants.kLeftJoystickDeadband)),
                                                                                maximumSpeed);

    swerveDrive.drive(scaledInputs,
    MathUtil.applyDeadband(angularVelocity, OperatorConstants.kRightJoystickDeadband) * -swerveDrive.getMaximumChassisAngularVelocity(),
      true, true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
  }
}
