// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

private final IntakeIO io;
IntakeIOInputs inputs = new IntakeIOInputs();


//---------------------------------------

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setIntakeVoltage(double voltage){
    io.setIntakeVoltage(voltage);
  }

  //---------------------------------------



  
  //---------------------------------------
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
  
  public double getCoralWristIntakeCurrent() {
    return inputs.coralWristVoltage;
  }
  //--------------------------------------
}
