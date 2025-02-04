// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class ElevatorIO extends SubsystemBase {
  public SparkMax leadMotor;
  public SparkMax followerMotor;
  Follower follower;
  
  SparkMaxConfig followerConfig = new SparkMaxConfig();
  SparkMaxConfig leadConfig = new SparkMaxConfig();
  
  Slot0Configs closedLoopConfigs;
  PositionVoltage m_request;

  /** Creates a new elevatorIO. */
  public ElevatorIO() {
    leadMotor = new SparkMax(MotorIds.kElevatorLeadMotor, MotorType.kBrushless);
    followerMotor = new SparkMax(MotorIds.kElevatorFollowMotor, MotorType.kBrushless);

    leadConfig.closedLoop.pid(0, 0, 0);
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
    return leadMotor.getEncoder().getPosition();
  }


  public double getVelocity() {
    // Get the velocity from the encoder
    return leadMotor.getEncoder().getVelocity();
  }


  public void resetPosition() {
    // Reset the encoder to the specified position
    leadMotor.getEncoder();
  }

  
  public void setPosition(double position) {
    leadMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }


  public void stop() {
    leadMotor.setVoltage(0);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
