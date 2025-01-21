// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class elevatorIO extends SubsystemBase {
  public SparkMax leadMotor;
  public SparkMax followerMotor;
  public RelativeEncoder encoder;

  SparkBaseConfig followerConfig = new SparkMaxConfig();
  SparkMaxConfig leadConfig = new SparkMaxConfig();

  /** Creates a new elevatorIO. */
  public elevatorIO() {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new SparkMax(MotorIds.kElevatorLeadMotor, MotorType.kBrushless);
    followerMotor = new SparkMax(MotorIds.kElevatorFollowMotor, MotorType.kBrushless);

      
      
    followerConfig.inverted(true);
    followerConfig.follow(leadMotor, true);
    followerConfig.idleMode(IdleMode.kBrake);

    followerMotor.configure(followerConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    leadConfig.idleMode(IdleMode.kBrake);
    leadConfig.closedLoop.pid(0, 0, 0);

    leadMotor.configure(leadConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    

    encoder = leadMotor.getEncoder();




  }

  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
  }


  public double getVelocity() {
    // Get the velocity from the encoder
    return encoder.getVelocity();
  }


  public void resetPosition() {
    // Reset the encoder to the specified position
    encoder.setPosition(0);
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
