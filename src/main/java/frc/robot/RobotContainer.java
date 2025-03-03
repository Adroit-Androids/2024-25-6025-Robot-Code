// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevator.ELevatorEnableManualControl;
import frc.robot.commands.Elevator.ElevatorAlgea1;
import frc.robot.commands.Elevator.ElevatorAlgea2;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorL1;
import frc.robot.commands.Elevator.ElevatorL2;
import frc.robot.commands.Elevator.ElevatorL3;
import frc.robot.commands.Intake.IntakeAlgea;
import frc.robot.commands.Intake.ShootAlgea;
import frc.robot.commands.Intake.ShootCoral;
import frc.robot.commands.Intake.ShootCoralSetTime;
import frc.robot.commands.Elevator.ElevatorL4;
import frc.robot.commands.Elevator.ElevatorNet;
import frc.robot.commands.SwerveDrive.AbsoluteDrive;
import frc.robot.commands.SwerveDrive.DriveTillSetTime;
import frc.robot.commands.SwerveDrive.TurnDrive;
import frc.robot.commands.SwerveDrive.Apriltag.TargetPoseAllignment;
import frc.robot.commands.SwerveDrive.CommandGroups.ReefAllignment;
import frc.robot.commands.Wrist.SetWristAngle;
import frc.robot.commands.Wrist.SetWristRest;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOHardware;
import frc.robot.subsystems.Endgame.Endgame;
import frc.robot.subsystems.Intake.IntakeIOHardware;
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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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

  public static ElevatorState currentElevatorState = ElevatorState.DOWN;
  public static ElevatorState lastElevatorState = ElevatorState.DOWN;
  public static boolean allignmentCommandTimerEnded = false;

  // Replace with CommandXboxController or CommandJoystick if needed
  public static final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  
  public static final CommandXboxController m_operatorController=
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...

  public static final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public static final Endgame m_endgame = new Endgame();
  public static final Elevator m_elevator = new Elevator(new ElevatorIOHardware());
  public static final Wrist m_wrist = new Wrist();
  public static final Limelight m_limelight = new Limelight(m_swerveDrive);
  public static final Intake m_intake = new Intake(new IntakeIOHardware());
  // public static final MusicPlayer m_musicPlayer = new MusicPlayer();
  // public static final intake m_intake = new intake(new IntakeIO());
  
  public static final AbsoluteDrive absoluteDriveCommand = new AbsoluteDrive(m_swerveDrive, m_driverController);
  public static final TurnDrive turnDriveCommand = new TurnDrive(m_swerveDrive, m_driverController);
  public SendableChooser<Command> autoChooser;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // // Configure named commands
    registerNamedCOmmands();

    
    //Configure the autochooser
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  public void registerNamedCOmmands(){
    new EventTrigger("ElevatorL4").onTrue(new ElevatorL4(m_elevator));
    NamedCommands.registerCommand("ShootCoral", new ShootCoralSetTime(m_intake, 0.5, 0.45));
    NamedCommands.registerCommand("RetrieveCoral", new ShootCoralSetTime(m_intake, 0.3, 0.35));
    NamedCommands.registerCommand("ElevatorDown", new ElevatorDown(m_elevator));
    NamedCommands.registerCommand("ElevatorL1", new ElevatorL1(m_elevator));
    NamedCommands.registerCommand("ElevatorL2", new ElevatorL2(m_elevator));
    NamedCommands.registerCommand("ElevatorL3", new ElevatorL3(m_elevator));
    NamedCommands.registerCommand("ElevatorL4", new ElevatorL4(m_elevator));
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
    m_driverController.L3().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setDefaultCommand(absoluteDriveCommand)));
    m_driverController.R3().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setDefaultCommand(turnDriveCommand)));
      //PoseLock

    //   //Coral Allignment 
    //
    // WARNING:
    //    FORWARD DISTANCE GIVEN INTO THE COMMAND MUST BE NEGATIVE 
    //    AND THE LEFT DISTANCE SHOULD BE AROUND THE -0.25 +0.25 MAX
    //
    m_driverController.circle().onTrue(new ReefAllignment(m_swerveDrive, m_limelight, 0.16, -1.0, 0.6, 0.9));
    m_driverController.square().onTrue(new ReefAllignment(m_swerveDrive, m_limelight, -0.20, -0.8, 0.6, 0.6));
    m_driverController.triangle().onTrue(new ReefAllignment(m_swerveDrive, m_limelight, 0.0, -0.9, 0.6, 0.6));


    m_operatorController.a().onTrue(new ElevatorL1(m_elevator));
    m_operatorController.a().onTrue(new ShootCoralSetTime(m_intake, 0.3, 0.35));
    m_operatorController.x().onTrue(new ElevatorL2(m_elevator));
    m_operatorController.x().onTrue(new ShootCoralSetTime(m_intake, 0.3, 0.35));
    m_operatorController.y().onTrue(new ElevatorL4(m_elevator));
    m_operatorController.y().onTrue(new ShootCoralSetTime(m_intake, 0.3, 0.35));
    m_operatorController.b().onTrue(new ElevatorL3(m_elevator));
    m_operatorController.b().onTrue(new ShootCoralSetTime(m_intake, 0.3, 0.35));

    m_operatorController.povUp().onTrue(new ElevatorAlgea2(m_elevator));
    m_operatorController.povDown().onTrue(new ElevatorAlgea1(m_elevator));
    m_operatorController.povLeft().onTrue(new ElevatorDown(m_elevator));
    m_operatorController.povRight().onTrue(new ElevatorNet(m_elevator));
    m_operatorController.povRight().onTrue(new SetWristAngle(m_wrist, 100));
    
    m_operatorController.back().onTrue(new ElevatorDown(m_elevator));

        //Operator Controls:
    m_driverController.R2().whileTrue(new ShootAlgea(m_intake));
    m_driverController.L2().whileTrue(new IntakeAlgea(m_intake, m_wrist, 63));
    m_driverController.L1().whileTrue(new IntakeAlgea(m_intake, m_wrist, 22.5));
    m_driverController.cross().whileTrue(new ShootCoral(m_intake));
    m_driverController.cross().onFalse(new ElevatorDown(m_elevator));
    // m_driverController.rightBumper().onFalse(new ElevatorDown(m_elevator));


    m_operatorController.leftStick().onTrue(new ELevatorEnableManualControl(m_elevator));

    // m_operatorController.rightTrigger(0.1).whileTrue(new RepeatCommand(new EndgameUp(m_endgame, m_operatorController.getRightTriggerAxis())));
    // m_operatorController.leftTrigger(0.1).whileTrue(new RepeatCommand(new EndgameDown(m_endgame, m_operatorController.getRightTriggerAxis())));
    // m_operatorController.rightTrigger().whileTrue(new EndgameUp(m_endgame, 0.7));
    // m_operatorController.leftTrigger().whileTrue(new EndgameUp(m_endgame, -1));
    // m_operatorController.rightBumper().whileTrue(new EndgameUp(m_endgame, 0.5));
    // m_operatorController.leftBumper().whileTrue(new EndgameUp(m_endgame, -0.5));

    //     // L1 state
    // m_operatorController.a().onTrue(new ElevatorL1(elevatorSubsystem));

    //     // L2 state
    // m_operatorController.x().onTrue(new ElevatorL2(elevatorSubsystem));

    //     // L3 state
    // m_operatorController.y().onTrue(new ElevatorL3(elevatorSubsystem));

    //     // L4 state
    // m_operatorController.b().onTrue(new ElevatorL0(elevatorSubsystem));

    //     // Elevator Down State
    // m_operatorController.start().onTrue(new ElevatorDown(elevatorSubsystem));

    

    Command driveRight = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(0.25, 0, 0)));
    m_driverController.povRight().whileTrue(new RepeatCommand(driveRight));
    
    Command driveLeft = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(-0.25, 0, 0)));
    m_driverController.povLeft().whileTrue(new RepeatCommand(driveLeft));

    Command driveForward = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(0, -0.25, 0)));
    m_driverController.povUp().whileTrue(new RepeatCommand(driveForward));

    Command driveBackward = new RunCommand(() -> m_swerveDrive.swerveDrive.drive(turnDriveCommand.getChassisSpeeds(0, 0.25, 0)));
    m_driverController.povDown().whileTrue(new RepeatCommand(driveBackward));
    
        //Coral Station State

        //Coral Intake

        //Algae Intake
    // m_operatorController.a().whileTrue(new RunCommand(() -> m_intake.setAlgaeIntakeVoltage(0.4), m_intake));
    // m_operatorController.b().whileTrue(new RunCommand(() -> m_intake.setAlgaeIntakeVoltage(-0.4), m_intake));

        //Algae Drop

        //Coral Drop

        //Debug
      m_driverController.touchpad().onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.resetOdometry(new Pose2d(m_swerveDrive.getPose().getX(), m_swerveDrive.getPose().getY(), new Rotation2d(Math.toRadians(180))))));
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
