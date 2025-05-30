// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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
  //boolean isValidID = false;
  double[] fidicualPose;
  int targetTimer = 0;
  double targetAngle = 0;
  double targetLeft;
  double targetForward;
  double angleError;
  double currentAngle;
  double minSpeed = 0.0;
  double pAdjustment = 0.0;
  double timer;
  double timerLimit = 0.25;
  double timerStartTimeFPGAT;

  double targetedTagId;
  
  // double kP = 1.0;
  
  boolean lastTimerStartedState = false;
  boolean timerStarted = false;

  PIDController angleVelocityController = new PIDController(1.75, 0.0, 0.0);
  PIDController leftVelocityController = new PIDController(1.3, 0.0, 0.0);
  PIDController forwardVelocityController = new PIDController(0.75, 0.0, 0.0);


  private double xTranslation = 0;
  private double yTranslation = 0;
  private double thetaTranslation = 0;
  /** Creates a new apriltagAllignment. */
  public TargetPoseAllignment(SwerveSubsystem swerveSubsystem, Limelight limelight, double leftDistance, double forwardDistance) {
    // Use addRequirements() here to declare subsystem dependencies.,
    addRequirements(swerveSubsystem);
    this.m_swerveDrive = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.swerveDrive;
    this.m_limelight = limelight;
    this.targetLeft = leftDistance;
    this.targetForward = forwardDistance;
    leftVelocityController.setTolerance(0.055);
    forwardVelocityController.setTolerance(0.35);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetedTagId = m_limelight.currentApriltagID;
    leftVelocityController.setSetpoint(targetLeft);
    forwardVelocityController.setSetpoint(targetForward);
    angleVelocityController.setSetpoint(0.0);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fidicualPose = m_limelight.getTargetPose2d();

    if (m_limelight.currentApriltagID == -1 && lastTimerStartedState == false) {
      timerStarted = true;
      startTimer();
    }

    if (timerStarted) {
      timer = Timer.getFPGATimestamp() - timerStartTimeFPGAT;
    }
    else {
      timer = 0;
    }

  if (targetedTagId == m_limelight.currentApriltagID){
    if (fidicualPose[2] != 0){
        xTranslation = forwardVelocityController.calculate(fidicualPose[2]);
        timerStarted = false;
    }
    if (fidicualPose[0] != 0){
        yTranslation = leftVelocityController.calculate(fidicualPose[0]);
    }
    thetaTranslation = angleVelocityController.calculate(fidicualPose[4]);
  }
    swerveDrive.drive(new ChassisSpeeds(xTranslation, -yTranslation,-Math.toRadians(thetaTranslation)));



    lastTimerStartedState = timerStarted;

    SmartDashboard.putNumber("Allignment speed", yTranslation);
    SmartDashboard.putBoolean("Command is finished", isFinished());
    SmartDashboard.putNumber("Angle Pid", pAdjustment);
    SmartDashboard.putNumber("Target angle", targetAngle);
    SmartDashboard.putNumber("Forward pid",  forwardVelocityController.calculate(fidicualPose[2]));
    SmartDashboard.putNumber("Current angle", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putNumber("Angle error", angleError);
    SmartDashboard.putNumber("Target left", targetLeft);
    SmartDashboard.putNumber("Target forward", targetForward);
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (timer > timerLimit) {
      RobotContainer.allignmentCommandTimerEnded = true;
    }
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    SmartDashboard.putNumber("timer", timer);
    if ( (forwardVelocityController.atSetpoint() && leftVelocityController.atSetpoint() && Math.abs(fidicualPose[4]) < 0.75) || timer > timerLimit){
      return true;
    }
    else {
      return false;
    }
  }

  public void startTimer() {
    timerStartTimeFPGAT = Timer.getFPGATimestamp();
  }
}
