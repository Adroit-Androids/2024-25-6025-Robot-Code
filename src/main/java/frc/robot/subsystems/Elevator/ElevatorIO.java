// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

  public interface ElevatorIO {
    
    @Autolog
    public class ElevatorIOInputs{
      public boolean elevatorMotorConnected = true;
      public boolean elevatorMotorFollowerConnected = true;
      public double motorCurrent = 0.0;
      public double voltageCurent = 0.0;
      public double elevatorPositionRad = 0.0;
      public double elevatorVelocity = 0.0;
    }
    
    public default double setPosition(double position) {}

    public default void setVoltage(double voltage) {}

    public default void resetPosition() {}
    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default double getVelocity() {
      return 0;
    }

    public default void setProfieldPIDValues(double kP, double kI, double kD) {}
  }
