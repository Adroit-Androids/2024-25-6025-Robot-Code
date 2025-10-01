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
    leadConfig.inverted(true);

    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.inverted(false);
    followerConfig.follow(leadMotor);

    leadMotor.configure(leadConfig, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
  }

  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.setVoltage(voltage);
  }

  public double getPosition() {
    // Get the position from the encoder
    return RobotContainer.m_endgame.getElevatorPosition();
  }


  public double getVelocity() {
    // Get the velocity from the encoder
    return leadMotor.getEncoder().getVelocity();
  }

  public double getLeadVoltage() {
    // Get the voltage from the lead motor
    return leadMotor.getBusVoltage();
  }

  public double getFollowerVoltage(){
    // Get the voltage from the follower motor
    return followerMotor.getBusVoltage();
  }

  public void resetPosition() {
    // Reset the encoder to the specified position
    leadMotor.getEncoder().setPosition(0.0);
  }

  public void stop() {
    leadMotor.stopMotor();
  }

  public void periodic() {
  }
}
