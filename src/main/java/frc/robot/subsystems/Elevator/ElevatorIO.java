// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.AutoLog;

  public interface ElevatorIO {

    @Autolog
    public class ElevatorIOInputs{
    // Inputs for the elevator
      public boolean motorLeaderConnected = true;
      public boolean motorFollowerConnected = true;

      public double motorCurrent = 0.0;
      public double voltageCurent = 0.0;
      public double voltageApplied[] = new double[] {};
      public double elevatorPositionRad = 0.0;
      public double elevatorVelocity = 0.0;
    }
    
    // Update the inputs for the elevator
    public default void updateInputs(ElevatorIOInputs inputs) {
    }
    
    // Set the voltage of the elevator
    public default void set(double voltage) {
    }

    // Set the current position of the elevator
    public default double setPosition(double position) {
    }

    // Get the current position of the elevator
     public default double getPosition() {
       return 0;
     }
     
     // Reset the position of the elevator
     public default void resetPosition() {
     }

    
    // Get the current velocity of the elevator
    public default double getVelocity() {
      return 0;
    }


    
    public default void setProfiledPID(double kP, double kI, double kD) {}
  }
