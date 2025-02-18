// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorTrapezoid;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  private final double maxVelocity = ElevatorTrapezoid.maxVelocity;
  private final double maxAcceleration = ElevatorTrapezoid.maxAcceleration;

  // PID and Feedforward constants
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.1;   // Static
  private static final double kV = 0.1;   // Velocity
  private static final double kG = 0.01;  // Gravity

  private final ProfiledPIDController pidController;
  private final ElevatorFeedforward feedforward;

  private DigitalInput magneticSwitchUpper = new DigitalInput(1);
  private DigitalInput magneticSwitchLower = new DigitalInput(2);

  private double targetPosition = 0.0; // Current target position

  // Constructor
  public Elevator(ElevatorIO io) {
    this.io = io;
    
    // Set up the PID controller with limits on position changes
    pidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    pidController.setTolerance(0.1); // Set tolerance for reaching the target position
    feedforward = new ElevatorFeedforward(kS, kG, kV); // Set up feedforward values
    feedforward.calculate(10, 20); // Calculate feedforward values    
    io.resetPosition(); // Initialize elevator position
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
    return io.getPosition();
  }

  // Get the current velocity of the elevator
  public double getVelocity() {
    return io.getVelocity();
  }

  // Move the elevator to the target position
  public void moveToPosition() {
    if (!pidController.atGoal()) {

      double pidOutput = pidController.calculate(getPosition());

      double feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);

      double voltage = pidOutput + feedforwardOutput;
      io.set(voltage);
    } else {
      io.stop();  // Stop the motor if the goal has been reached
    }
  }

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
  
  
  @Override
  public void periodic() {
    // If the upper or lower limit switches are triggered, stop the elevator and reset position if necessary
    if (magneticSwitchUpper.get()) {
      setPosition(io.getPosition());
      moveToPosition();
    } else if (magneticSwitchLower.get()) {
      io.resetPosition();
      setPosition(io.getPosition());
      moveToPosition();
    } else {
      moveToPosition();
    }
    inputs.elevatorVelocity = io.getVelocity();
    inputs.voltageCurent = io.getVelocity();

    io.updateInputs(inputs);
  }
}