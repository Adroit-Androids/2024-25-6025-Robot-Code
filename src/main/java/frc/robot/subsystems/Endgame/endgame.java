// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Endgame;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class endgame extends SubsystemBase {
  /** Creates a new EndgameIO. */
  TalonSRX EndgameLeft;
  TalonSRX EndgameRight;

  public endgame() {
    EndgameLeft = new TalonSRX(20);
    EndgameRight = new TalonSRX(21);

    EndgameLeft.configFactoryDefault();
    EndgameRight.configFactoryDefault();
    EndgameLeft.setNeutralMode(NeutralMode.Brake);
    EndgameRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed){
    EndgameLeft.set(ControlMode.PercentOutput, speed);
    EndgameRight.set(ControlMode.PercentOutput, speed);
  }

  public void setSpeedZero(){
    EndgameLeft.set(ControlMode.PercentOutput, 0);
    EndgameRight.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
