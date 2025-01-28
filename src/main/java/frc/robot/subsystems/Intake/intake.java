// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class intake extends SubsystemBase {

IntakeIO io;
IntakeIOInputs inputs = new IntakeIOInputs();


//---------------------------------------

  public intake(IntakeIO io) {
    this.io = io;
  }

  public void setAlgaeVoltage(double voltage){
    io.setAlgaeVoltage(voltage);
  }

  public void setCoralIntakeVoltage(double voltage){
    io.setCoralIntakeVoltage(voltage);
  }

  //---------------------------------------

  private double targetPosition = 0.0;

  public void setWristPositionByDegrees(double position) {
    targetPosition = Math.toRadians(position);
  }

  public double getTargetWristPosition() {
    return targetPosition;
  }

  
  //---------------------------------------
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
  
  public double getCoralWristIntakeCurrent() {
    return inputs.coralWristCurrent;
  }
  
  public double getWristPosition(){ 
    return inputs.coralWristPosition;
  }

  public void wristAngle(double position) {
    io.wristAngle(position);;
  }

  public void setWristVoltage(double voltage) {
    io.setWristVoltage(voltage);
  }
  
  //---------------------------------------
  public void resetAngle(double Radians) {}
}