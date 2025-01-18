// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive.Poselock;
import frc.robot.commands.SwerveDrive.absoluteDrive;
import frc.robot.subsystems.Swerve.swerveSubsystem;
import frc.robot.subsystems.Elevator.elevator;
import frc.robot.subsystems.Elevator.elevatorIO;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...

  public static final swerveSubsystem m_swerveDrive = new swerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public static final absoluteDrive m_absoluteDriveCommand = new absoluteDrive(m_swerveDrive, m_driverController);
  public static final elevator m_elevator = new elevator(new elevatorIO());

  //public static final telescopicArm m_telescopicArm = new telescopicArm();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    new EventTrigger("Coral_Placement").onTrue(Commands.print("1"));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.a().whileTrue(new Poselock(m_swerveDrive));

        // L1 state
    Command liftToL1Command = new RunCommand(() -> m_elevator.setPosition(L1_HEIGHT), m_elevator);
    m_driverController.a().onTrue(liftToL1Command);

        // L2 state
    Command liftToL2Command = new RunCommand(() -> m_elevator.setPosition(L2_HEIGHT), m_elevator);
    m_driverController.b().onTrue(liftToL2Command);

        // L3 state
    Command liftToL3Command = new RunCommand(() -> m_elevator.setPosition(L3_HEIGHT), m_elevator);
    m_driverController.x().onTrue(liftToL3Command);

        // L4 state
    Command liftToL4Command = new RunCommand(() -> m_elevator.setPosition(L4_HEIGHT), m_elevator);
    m_driverController.y().onTrue(liftToL4Command);


    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    // An example command will be run in autonomous
  }
}
}
