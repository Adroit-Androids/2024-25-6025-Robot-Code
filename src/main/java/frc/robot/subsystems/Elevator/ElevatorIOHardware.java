// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotContainer;
import frc.robot.Constants.MotorIds;

public class ElevatorIOHardware implements ElevatorIO {
  public SparkMax leadMotor;
  public SparkMax followerMotor;
  
  SparkMaxConfig followerConfig;
  SparkMaxConfig leadConfig;
  
  PositionVoltage m_request;

  public ElevatorIOHardware() {
    leadMotor = new SparkMax(MotorIds.kElevatorLeadMotor, MotorType.kBrushless);
    followerMotor = new SparkMax(MotorIds.kElevatorFollowMotor, MotorType.kBrushless);

    followerConfig = new SparkMaxConfig();
    leadConfig = new SparkMaxConfig();

    leadConfig.idleMode(IdleMode.kBrake);
    leadConfig.inverted(false);

    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.inverted(true);
    followerConfig.follow(leadMotor);

    leadMotor.configure(leadConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
    followerMotor.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
  }

  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  public double getPosition() {
    // Get the position from the encoder
    return RobotContainer.m_endgame.getElevatorPosition();
  }


  public double getVelocity() {
    // Get the velocity from the encoder
    return leadMotor.getEncoder().getVelocity();
  }

  public double getVoltage() {
    // Get the voltage from the motor
    return leadMotor.getBusVoltage();
  }


  public void resetPosition() {
    // Reset the encoder to the specified position
    leadMotor.getEncoder().setPosition(0.0);
  }

  public void stop() {
    leadMotor.setVoltage(0);
  }

  public void periodic() {
  }
}
