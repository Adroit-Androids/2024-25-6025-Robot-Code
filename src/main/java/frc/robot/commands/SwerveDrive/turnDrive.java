// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve.swerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class turnDrive extends Command {
  swerveSubsystem m_swerveSubsystem;
  SwerveDrive swerveDrive;
  CommandXboxController robotController;


  /** Creates a new turnDrive. */
  public turnDrive(swerveSubsystem m_swerveSubsystem, CommandXboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.swerveDrive = m_swerveSubsystem.swerveDrive;
    this.robotController = m_driverController;
  }

    public ChassisSpeeds getChassisSpeeds(double translationX, double translationY, double angularVelocity){
    
    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(MathUtil.applyDeadband(-translationY, OperatorConstants.kLeftJoystickDeadband),
                                                                                MathUtil.applyDeadband(-translationX, OperatorConstants.kLeftJoystickDeadband)),
                                                                                m_swerveSubsystem.maximumSpeed);

    ChassisSpeeds chassisSpeeds = swerveDrive.swerveController.getRawTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
    MathUtil.applyDeadband(angularVelocity, OperatorConstants.kRightJoystickDeadband) * -swerveDrive.getMaximumChassisAngularVelocity());

    return chassisSpeeds;

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.setChassisSpeeds(getChassisSpeeds(robotController.getLeftX(), robotController.getLeftY(), robotController.getRightX()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
