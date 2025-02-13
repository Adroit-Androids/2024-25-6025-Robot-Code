// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorL1;
import frc.robot.commands.Elevator.ElevatorL2;
import frc.robot.commands.Elevator.ElevatorL3;
import frc.robot.commands.Elevator.ElevatorL4;
import frc.robot.commands.SwerveDrive.AbsoluteDrive;
import frc.robot.commands.SwerveDrive.Apriltag.ApriltagAllignment;
import frc.robot.commands.SwerveDrive.Apriltag.ApriltagDistanceAndCoralAllignment;
import frc.robot.commands.SwerveDrive.TurnDrive;
import frc.robot.commands.SwerveDrive.CommandGroups.ReefAllignment;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.MusicPlayer;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOHardware;
import frc.robot.subsystems.Elevaotr.ElevatorIO;
import frc.robot.subsystems.Endgame.Endgame;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Limelight.Limelight;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  public static double lastReadTxTarget;
  public static double currentTargetID;

  private final double LEFT_TX = -6;
  private final double RIGHT_TX = 6;
  private final double MIDDLE_TX = 0; 

  private final double PROCESSOR_HEIGHT = 0;
  private final double SOURCE_HEIGHT = 8.75;
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
  
  public static final CommandXboxController m_operatorController=
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...

  public static final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public static final Elevator elevatorSubsystem = new Elevator(new ElevatorIO());
  public static final Endgame m_endgame = new Endgame();
  public static final Limelight m_limelight = new Limelight(m_swerveDrive);
  public static final MusicPlayer m_musicPlayer = new MusicPlayer();
  // public static final intake m_intake = new intake(new IntakeIO());
  
  public static final AbsoluteDrive absoluteDriveCommand = new AbsoluteDrive(m_swerveDrive, m_driverController);
  public static final TurnDrive turnDriveCommand = new TurnDrive(m_swerveDrive, m_driverController);
  public static final ApriltagAllignment apriltagAllignmentCommand = new ApriltagAllignment(m_swerveDrive, m_limelight);
  public SendableChooser<Command> autoChooser;
  //public static final telescopicArm m_telescopicArm = new telescopicArm();




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure named commands
    NamedCommands.registerCommand("Reef_Position_Right", new ReefAllignment(m_swerveDrive, m_limelight, 10.0, 8.0));

    
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
    m_driverController.leftBumper().onTrue(new ReefAllignment(m_swerveDrive, m_limelight, -16.0, 10.0));
    m_driverController.rightBumper().onTrue(new ReefAllignment(m_swerveDrive, m_limelight, 16.0, 10.0));
    m_driverController.rightTrigger().whileTrue(new ApriltagDistanceAndCoralAllignment(m_swerveDrive, m_limelight, 16.0, 10.0, true));
      //Coral Station Allignment

      //Processor Allignment

      //Endgame

        //Operator Controls:

        // L1 state
    m_operatorController.a().onTrue(new ElevatorL1(elevatorSubsystem));

        // L2 state
    m_operatorController.x().onTrue(new ElevatorL2(elevatorSubsystem));

        // L3 state
    m_operatorController.y().onTrue(new ElevatorL3(elevatorSubsystem));

        // L4 state
    m_operatorController.b().onTrue(new ElevatorL4(elevatorSubsystem));

        // Elevator Down State
    m_operatorController.start().onTrue(new ElevatorDown(elevatorSubsystem));

    

    Command driveRight = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(1, 0, 0)));
    m_driverController.povRight().whileTrue(new RepeatCommand(driveRight));
    
    Command driveLeft = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(-1, 0, 0)));
    m_driverController.povLeft().whileTrue(new RepeatCommand(driveLeft));

    Command driveForward = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(0, -1, 0)));
    m_driverController.povUp().whileTrue(new RepeatCommand(driveForward));

    Command driveBackward = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(0, 1, 0)));
    m_driverController.povDown().whileTrue(new RepeatCommand(driveBackward));
    
        //Coral Station State

        //Coral Intake

        //Algae Intake

        //Algae Drop

        //Coral Drop

        //Debug
      m_driverController.start().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.resetOdometry(new Pose2d(7.215, 4.001, new Rotation2d(Math.toRadians(180))))));
        
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
