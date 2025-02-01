// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class musicPlayer extends SubsystemBase {
  TalonFX leadMotor;
  TalonFX bassMotor;

  Orchestra orchestra;


  /** Creates a new musicPlayer. */
  public musicPlayer() {
    leadMotor = new TalonFX(10);
    bassMotor = new TalonFX(11);
    
    orchestra = new Orchestra();
    
    orchestra.loadMusic("kufi.chrp");

    orchestra.addInstrument(leadMotor, 1);
    orchestra.addInstrument(bassMotor, 2);

    orchestra.play();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
