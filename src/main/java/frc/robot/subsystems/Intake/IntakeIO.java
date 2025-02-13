// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class IntakeIO extends SubsystemBase {
  TalonSRX coralIntake;
  TalonFX coralWrist;

  TalonFXConfiguration coralWristConfig;

  TalonSRX algeaIntake;

  PositionVoltage m_request;
  Slot0Configs closedLoopConfigs;

@AutoLog
  public static class IntakeIOInputs {
    public double coralWristVoltage = 0.0;
    public double coralWristVelocity = 0.0;
    public double coralWristPosition = 0.0;
  }



  public IntakeIO() {
    coralIntake = new TalonSRX(0);
    coralWrist = new TalonFX(1);

    coralWristConfig = new TalonFXConfiguration();
    closedLoopConfigs = new Slot0Configs();

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

  public void setAlgaeVoltage(double voltage) {
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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
