// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.MotorIds;

public class IntakeIOHardware implements IntakeIO {
  TalonSRX coralIntake;
  TalonFX coralWrist;

  TalonFXConfiguration coralWristConfig;

  TalonSRX algeaIntake;

  PositionVoltage m_request;
  Slot0Configs closedLoopConfigs;

  public IntakeIOHardware() {
    coralIntake = new TalonSRX(0);
    coralWrist = new TalonFX(1);

    coralWristConfig = new TalonFXConfiguration();

    m_request = new PositionVoltage(0);
    m_request.Slot = 0;

    algeaIntake = new TalonSRX(MotorIds.kAlgeaIntakeMotor);

    algeaIntake.configFactoryDefault();
    algeaIntake.setNeutralMode(NeutralMode.Brake);
    coralIntake.configFactoryDefault();
    coralIntake.setNeutralMode(NeutralMode.Brake);
    
    coralWristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralWristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    closedLoopConfigs.withKP(0.1);
    closedLoopConfigs.withKI(0.0);
    closedLoopConfigs.withKD(0.0);

    coralWrist.getConfigurator().apply(coralWristConfig);
    coralWrist.getConfigurator().apply(closedLoopConfigs);

  }


  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristVoltage = coralWrist.getMotorVoltage().getValueAsDouble();
    inputs.coralWristVelocity = coralWrist.getVelocity().getValueAsDouble();
    inputs.coralWristPosition = coralWrist.getPosition().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    algeaIntake.set(TalonSRXControlMode.Current, voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.set(TalonSRXControlMode.Current, voltage);
  }

  public void adjustAngle(double angleRadians) {
    coralWrist.setPosition(coralWrist.getPosition().getValueAsDouble() + angleRadians);
  }

  public void wristAngle(double position) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.setControl(m_request.withPosition(position));
    
  }

  public double getWristPosition() {
    return coralWrist.getPosition().getValueAsDouble();
  }

  public void setWristVoltage(double voltage) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.set(voltage);
  }
}
