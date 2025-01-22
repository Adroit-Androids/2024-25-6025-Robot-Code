// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class elevatorIO extends SubsystemBase {
  public TalonFX leadMotor;
  public TalonFX followerMotor;
  Follower follower;
  
  TalonFXConfiguration followerConfig = new TalonFXConfiguration();
  TalonFXConfiguration leadConfig = new TalonFXConfiguration();
  
  Slot0Configs closedLoopConfigs;
  PositionVoltage m_request;

  /** Creates a new elevatorIO. */
  public elevatorIO() {
    leadMotor = new TalonFX(MotorIds.kElevatorLeadMotor);
    followerMotor = new TalonFX(MotorIds.kElevatorFollowMotor);

    closedLoopConfigs = new Slot0Configs();
    m_request = new PositionVoltage(0).withSlot(0);
      
    leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    closedLoopConfigs.kG = 0.5;
    closedLoopConfigs.kP = 0.00001;

    leadMotor.getConfigurator().apply(followerConfig);
    leadMotor.getConfigurator().apply(closedLoopConfigs);

    followerMotor.setControl(new Follower(leadMotor.getDeviceID(), true));
    







  }

  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  public double getPosition() {
    // Get the position from the encoder
    return leadMotor.getPosition().getValueAsDouble();
  }


  public double getVelocity() {
    // Get the velocity from the encoder
    return leadMotor.getVelocity().getValueAsDouble();
  }


  public void resetPosition() {
    // Reset the encoder to the specified position
    leadMotor.setPosition(0);
  }

  
  public void setPosition(double position) {
    leadMotor.setControl(m_request.withPosition(position));
  }


  public void stop() {
    leadMotor.setVoltage(0);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
