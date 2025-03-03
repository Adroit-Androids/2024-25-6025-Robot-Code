// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ElevatorTrapezoid;
import frc.robot.commands.Elevator.ElevatorDown;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  private final double maxVelocity = ElevatorTrapezoid.maxVelocity;
  private final double maxAcceleration = ElevatorTrapezoid.maxAcceleration;

  // PID and Feedforward constants
  private static final double kP = 4.25;
  private static final double kI = 0.5;
  private static final double kD = 0.0;
  private static final double kS = ((1.2 - 0.1) / 2);   // Static
  private static final double kV = 0.0;   // Velocity
  private static final double kG = (1.2 + 0.1) / 2;  // Gravity
  public double errorTolerance = 0.1;

  public ProfiledPIDController pidController;
  public ElevatorFeedforward feedforward;

  private double targetPosition = 0.0; // Current target position

  public boolean isManualControl = false;
  public boolean isAtSetpoint = false;
  public boolean hasCommandRunning;

  // Constructor
  public Elevator(ElevatorIO io) {
    this.io = io;
    
    // Set up the PID controller with limits on position changes
    pidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    pidController.setTolerance(errorTolerance); // Set tolerance for reaching the target position
    pidController.setIZone(0.5);
    feedforward = new ElevatorFeedforward(kS, kG, kV); // Set up feedforward values 
    //setDefaultCommand(new ElevatorDown(this));
    io.resetPosition(); // Initialize elevator position
    RobotContainer.currentElevatorState = ElevatorState.DOWN;
    setPosition(0);
  }

  public void setVoltage(double voltage){
    io.set(voltage);
  }

  // Set the elevator target position
  public void setPosition(double targetPosition) {
    if (this.targetPosition != targetPosition) {  // Only update if the target position changes
      this.targetPosition = targetPosition;
      pidController.setGoal(targetPosition);  // Update PID goal
    }
  }

  // Get the current position of the elevator
  public double getPosition() {
    return RobotContainer.m_endgame.getElevatorPosition() / 4096;
  }

  // Get the current velocity of the elevator
  public double getVelocity() {
    return RobotContainer.m_endgame.getElevatorSpeed();
  }

  // Move the elevator to the target position
  public void moveToPosition() {

    double pidOutput = pidController.calculate(getPosition(), targetPosition);

    double feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);

    if (pidController.atSetpoint() && RobotContainer.currentElevatorState == ElevatorState.DOWN) {
      setPosition(0.0);
    }

    double voltage = pidOutput + feedforwardOutput;
    io.set(voltage);

  }

//SysIdRoutine routine = new SysIdRoutine(null, null);

  public void movetoVelocity(double velocity) {
  double feedforwardOutput = feedforward.calculate(velocity);
  SmartDashboard.putNumber("Elevator target speed", velocity);
   io.set(feedforwardOutput);
  }

  // Stop the elevator
  public void stop() {
    io.stop();
  }

  // Reset the elevator position
  public void resetPosition() {
    io.resetPosition();
  }
  
  public boolean getIstAtSetpoint(){
    return isAtSetpoint;
  }
  
  @Override
  public void periodic() {
    if (Math.abs(getPosition() - targetPosition) < errorTolerance){
      isAtSetpoint = true;
    }
    else {
      isAtSetpoint = false;
    }
    if (!isManualControl){
      if (!isAtSetpoint){
        moveToPosition();
      }
      else {
        if (RobotContainer.currentElevatorState == ElevatorState.DOWN){
          io.stop();
        }
        else {
          io.set(kG);
        }
      }
    }
    else {
    double controlOutput = MathUtil.applyDeadband(RobotContainer.m_operatorController.getLeftY() * -1, 0.05) * 2;
    double feedforwardOutput = feedforward.calculate(controlOutput);
    setVoltage(controlOutput + feedforwardOutput);
    }


    inputs.elevatorVelocity = RobotContainer.m_endgame.getElevatorSpeed();
    inputs.voltageCurent = io.getVelocity();

    if (getCurrentCommand() != null){
      SmartDashboard.putString("Current Elevator Command", getCurrentCommand().getName());
      hasCommandRunning = true;
    }
    else {
      hasCommandRunning = false;
    }

    SmartDashboard.putBoolean("Elevator Has Command Running", hasCommandRunning);
    SmartDashboard.putNumber("Elevator position meter", getPosition());
    SmartDashboard.putNumber("Elevator velocity meter", getVelocity());
    SmartDashboard.putNumber("Elevator encoder value", RobotContainer.m_endgame.getElevatorPosition());
    SmartDashboard.putNumber("Elevator setpoint", pidController.getGoal().position);
    SmartDashboard.putNumber("Elevator voltage", io.getVoltage());
    SmartDashboard.putNumber("Elevator pid output", pidController.calculate(getPosition(), targetPosition));
    SmartDashboard.putBoolean("Elevator at setpoint", isAtSetpoint);
    SmartDashboard.putNumber("Elevator feedforward output", feedforward.calculate(pidController.getSetpoint().velocity));
    io.updateInputs(inputs);
  }
}