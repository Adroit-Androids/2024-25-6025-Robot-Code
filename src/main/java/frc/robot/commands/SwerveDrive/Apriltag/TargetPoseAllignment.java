// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive.Apriltag;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetPoseAllignment extends Command {
  SwerveSubsystem m_swerveDrive;
  SwerveDrive swerveDrive;
  Limelight m_limelight;
  int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  boolean isValidID = false;
  double[] fidicualPose;
  int targetTimer = 0;
  double targetAngle = 0;
  double targetLeft;
  double targetForward;
  double angleError;
  double currentAngle;
  double minSpeed = 0.0;
  double pAdjustment = 0.0;

  double kP = 4.5;

  PIDController leftVelocityController = new PIDController(2.75, 0.0, 0.04);
  PIDController forwardVelocityController = new PIDController(2.0, 0.0, 0.01);

  private double xTranslation = 0;
  private double yTranslation = 0;
  /** Creates a new apriltagAllignment. */
  public TargetPoseAllignment(SwerveSubsystem swerveSubsystem, Limelight limelight, double leftDistance, double forwardDistance) {
    // Use addRequirements() here to declare subsystem dependencies.,
    addRequirements(swerveSubsystem);
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
    this.targetLeft = leftDistance;
    this.targetForward = forwardDistance;
    leftVelocityController.setTolerance(0.015);
    forwardVelocityController.setTolerance(0.03);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (double i :validIDs){
      if (m_limelight.currentApriltagID == i){
        isValidID = true;
        RobotContainer.currentTargetID = m_limelight.currentApriltagID;
      }
    
      leftVelocityController.setSetpoint(targetLeft);
      forwardVelocityController.setSetpoint(targetForward);
    }

    // if (isValidID){

    //   if (m_limelight.currentApriltagID == 21 || m_limelight.currentApriltagID == 7){
    //     targetAngle = 0;
    //   }
    //   if (m_limelight.currentApriltagID == 22 || m_limelight.currentApriltagID == 6){
    //     targetAngle = 60;
    //   }
    //   if (m_limelight.currentApriltagID == 17 || m_limelight.currentApriltagID == 11){
    //     targetAngle = 120;
    //   }
    //   if (m_limelight.currentApriltagID == 18 || m_limelight.currentApriltagID == 10){
    //     targetAngle = 180;
    //   }
    //   if (m_limelight.currentApriltagID == 19 || m_limelight.currentApriltagID == 9){
    //     targetAngle = 240;
    //   }
    //   if (m_limelight.currentApriltagID == 20 || m_limelight.currentApriltagID == 8){
    //     targetAngle = 300;
    //   }
    // }

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fidicualPose = m_limelight.getTargetPose2d();
    currentAngle = fidicualPose[4];



    angleError = targetAngle - currentAngle;
    pAdjustment = angleError * kP;
    if (angleError < -180){
      angleError += 360;
    }

    // if (Math.abs(angleError) < 1){
    //   angleError = 0;
    // }
    
    // Increase targetTimer if we are within a certain range of our target angle
    // if (Math.abs(angleError) <= 1.0){
    //   ++targetTimer;
    // }
    // else{
    //   targetTimer = 0;
    // }

    if (m_limelight.currentApriltagID == RobotContainer.currentTargetID){
      RobotContainer.lastReadTxTarget = m_limelight.tx;
    }
    // if (!xVelocityController.atSetpoint()){
    //   xTranslation = xVelocityController.calculate(fidicualPose[2], targetY);
    // }

    // if (!yVelocityController.atSetpoint()) {
    //   yTranslation = yVelocityController.calculate(fidicualPose[0], targetX);
    // }
  if (fidicualPose[2] != 0){
      xTranslation = forwardVelocityController.calculate(fidicualPose[2]);
  }
  if (fidicualPose[0] != 0){
      yTranslation = leftVelocityController.calculate(fidicualPose[0]);
  }
  if (forwardVelocityController.atSetpoint()){
    xTranslation = 0;
  }
    // swerveDrive.drive(new Translation2d(0 * -xTranslation, -yTranslation),
    //                   0 * Math.toRadians(pAdjustment),
    //                   false, false);
    swerveDrive.drive(new ChassisSpeeds(0 * xTranslation, -yTranslation, -Math.toRadians(pAdjustment)));

    SmartDashboard.putBoolean("Command is finished", isFinished());
    SmartDashboard.putNumber("Target angle", targetAngle);
    SmartDashboard.putNumber("Current angle", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putNumber("Angle error", angleError);
    SmartDashboard.putNumber("Target left", targetLeft);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (leftVelocityController.atSetpoint()){
      return true;
    }
    return false;
    // if (forwardVelocityController.atSetpoint() && leftVelocityController.atSetpoint() && Math.abs(angleError) < 0.5){
    //   return true;
    // }
    // else {
    //   return false;
    // }
  }
}
