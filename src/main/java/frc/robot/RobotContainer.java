// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive.absoluteDrive;
import frc.robot.commands.SwerveDrive.apriltagAllignment;
import frc.robot.commands.SwerveDrive.turnDrive;
import frc.robot.commands.SwerveDrive.CommandGroups.reefAllignmentRight;
import frc.robot.subsystems.Swerve.swerveSubsystem;
import frc.robot.subsystems.musicPlayer;
import frc.robot.subsystems.Elevator.elevator;
import frc.robot.subsystems.Elevator.elevatorIO;
import frc.robot.subsystems.Endgame.endgame;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.intake;
import frc.robot.subsystems.Limelight.limelight;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final double PROCESSOR_HEIGHT = 0;
  private final double SOURCE_HEIGHT = 8.75;
  private final double L1_HEIGHT = 3;
  private final double L2_HEIGHT = 5.5;
  private final double L3_HEIGHT = 21.5;
  private final double L4_HEIGHT = 52.5;
  private final double TOP_ALGAE_HEIGHT = 40;

  private final double PROCESSOR_ANGLE = 0;
  private final double SOURCE_ANGLE = 0.15;
  private final double L1_ANGLE = 0.3;
  private final double L2_ANGLE = 0.225;
  private final double L3_ANGLE = 0.225;
  private final double L4_ANGLE = 0.26;
  private final double TOP_ALGAE_ANGLE = 0;

  // Replace with CommandXboxController or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  public static final CommandXboxController m_operatorContorller=
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...

  public static final swerveSubsystem m_swerveDrive = new swerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public static final elevator m_elevator = new elevator(new elevatorIO());
  public static final endgame m_endgame = new endgame();
  public static final limelight m_limelight = new limelight();
  public static final musicPlayer m_musicPlayer = new musicPlayer();
  // public static final intake m_intake = new intake(new IntakeIO());
  
  public static final absoluteDrive absoluteDriveCommand = new absoluteDrive(m_swerveDrive, m_driverController);
  public static final turnDrive turnDriveCommand = new turnDrive(m_swerveDrive, m_driverController);
  public static final apriltagAllignment apriltagAllignmentCommand = new apriltagAllignment(m_swerveDrive, m_limelight);
  public SendableChooser<Command> autoChooser;
  //public static final telescopicArm m_telescopicArm = new telescopicArm();




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    new EventTrigger("Coral_Placement").onTrue(Commands.print("1"));
    
    //Configure the autochooser
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandXboxController
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.a().whileTrue(new Poselock(m_swerveDrive))
      
    //Driver Controls:

      //FieldRelative
    m_driverController.leftStick().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setDefaultCommand(absoluteDriveCommand)));
      //RobotRelative
    m_driverController.rightStick().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setDefaultCommand(turnDriveCommand)));
      //PoseLock

      //Coral Allignment
    m_driverController.leftBumper().onTrue(new reefAllignmentRight(m_swerveDrive, m_limelight));
      //Coral Station Allignment

      //Processor Allignment

      //Endgame

        //Operator Controls:

        // L1 state
    Command liftToL1Command = new RunCommand(() -> m_elevator.setPosition(L1_HEIGHT), m_elevator);
    m_driverController.a().onTrue(liftToL1Command);

        // L2 state
    Command liftToL2Command = new RunCommand(() -> m_elevator.setPosition(L2_HEIGHT), m_elevator);
    m_driverController.x().onTrue(liftToL2Command);

        // L3 state
    Command liftToL3Command = new RunCommand(() -> m_elevator.setPosition(L3_HEIGHT), m_elevator);
    m_driverController.y().onTrue(liftToL3Command);

        // L4 state
    Command liftToL4Command = new RunCommand(() -> m_elevator.setPosition(L4_HEIGHT), m_elevator);
    m_driverController.b().onTrue(liftToL4Command);


    

    Command setDriveBy0Degrees = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(absoluteDriveCommand.getChassisSpeeds(1.0,0.0,0.0,0.0)));
    m_driverController.povRight().whileTrue(setDriveBy0Degrees);
    
    Command setDriveBy180Degrees = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(absoluteDriveCommand.getChassisSpeeds(-1.0,0.0,0.0,0.0)));
    m_driverController.povLeft().whileTrue(setDriveBy180Degrees);

    Command setDriveBy90Degrees = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(absoluteDriveCommand.getChassisSpeeds(0.0,1.0,0.0,0.0)));
    m_driverController.povUp().whileTrue(setDriveBy90Degrees);

    Command setDriveBy270Degrees = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(absoluteDriveCommand.getChassisSpeeds(0.0,-1.0,0.0,0.0)));
    m_driverController.povDown().whileTrue(setDriveBy270Degrees);
    
        //Coral Station State

        //Coral Intake

        //Algae Intake

        //Algae Drop

        //Coral Drop

        //Debug
      m_driverController.start().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.resetOdometry(new Pose2d(5, 1, new Rotation2d(0)))));
        
//--------------------------------------------------------------------------------------------------------------

    //    // Intake L1 State
    // Command intakeFromL1Command = new RunCommand(() -> m_intake.setWristPositionByDegrees(L1_ANGLE), m_intake);
    // m_driverController.cross().onTrue(intakeFromL1Command);


    //    // Intake L2 State
    // Command intakeFromL2Command = new RunCommand(() -> m_intake.setWristPositionByDegrees(L2_ANGLE), m_intake);
    // m_driverController.cross().onTrue(intakeFromL2Command);


    //   // Intake L3 State
    // Command intakeFromL3Command = new RunCommand(() -> m_intake.setWristPositionByDegrees(L3_ANGLE), m_intake);
    // m_driverController.cross().onTrue(intakeFromL3Command);


    //   // Intake L4 State
    // Command intakeFromL4Command = new RunCommand(() -> m_intake.setWristPositionByDegrees(L4_ANGLE), m_intake);
    // m_driverController.cross().onTrue(intakeFromL4Command);


    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}
}
