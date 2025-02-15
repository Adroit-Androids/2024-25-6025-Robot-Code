// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeConnected = true;
    public double voltageCurent = 0.0;
    
    public double coralWristVoltage = 0.0;
    public double coralWristVelocity = 0.0;
    public double coralWristPosition = 0.0;
  }


  public default void updateInputs(IntakeIOInputs inputs) {}

  // Set X Voltage
  public default void setWristVoltage(double voltage) {}
  public default void setAlgaeIntakeVoltage(double voltage) {}
  public default void setCoralIntakeVoltage(double voltage) {}

  public default void adjustAngle(double angle) {}
  public default void wristAngle(double position) {}

  public default void setCoralWristPosition(double position, double ffvoltage) {}

  public default void stop() {}
  }
