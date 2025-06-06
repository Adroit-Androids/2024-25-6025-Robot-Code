// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Endgame;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class Endgame extends SubsystemBase {
  /** Creates a new EndgameIO. */
  TalonSRX EndgameLeft;
  //TalonSRX EndgameRight;

  public Endgame() {
    EndgameLeft = new TalonSRX(MotorIds.kEndgameLeftMotor);
    //EndgameRight = new TalonSRX(MotorIds.kEndgameRightMotor);

    //EndgameRight.setInverted(false);
    
    EndgameLeft.configFactoryDefault();
    //EndgameRight.configFactoryDefault();
    //EndgameLeft.configPeakCurrentLimit(120);
    //EndgameRight.configPeakCurrentLimit(120);
    EndgameLeft.configVoltageCompSaturation(12.5);
    //EndgameRight.configVoltageCompSaturation(12.5);
    EndgameLeft.setNeutralMode(NeutralMode.Brake);
    EndgameLeft.configFactoryDefault();
    //EndgameRight.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * For electrical issues our encoder for our elevator had to be wired to the endgames talon srx.
   * So we get it from our endgame subsytem
   * 
   * @return Elevator sensor position
   */
  public double getElevatorPosition(){
    return EndgameLeft.getSelectedSensorPosition();
  }

  public double getElevatorSpeed(){
    return EndgameLeft.getSelectedSensorVelocity();
  }

  public void resetElevatorEncoder(){
    EndgameLeft.setSelectedSensorPosition(0.0);
  }

  public void setSpeed(double speed){
    EndgameLeft.set(ControlMode.PercentOutput, speed);
    //EndgameRight.set(ControlMode.PercentOutput, speed);
  }

  public void setSpeedZero(){
    EndgameLeft.set(ControlMode.PercentOutput, 0.0);
    //EndgameRight.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
