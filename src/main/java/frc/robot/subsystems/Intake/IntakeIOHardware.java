// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.Constants.MotorIds;

public class IntakeIOHardware implements IntakeIO {
  VictorSPX intakeMotor;

  TalonFXConfiguration coralWristConfig;


  PositionVoltage m_request;
  Slot0Configs closedLoopConfigs;

  public IntakeIOHardware() {
    intakeMotor = new VictorSPX(MotorIds.kIntakeMotor);


    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(12);
  }

  public void setIntakeVoltage(double voltage) {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, voltage);
  }

}
