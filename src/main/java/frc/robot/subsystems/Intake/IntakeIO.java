// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIO extends SubsystemBase {
  TalonSRX coralIntake;
  SparkMax coralWrist;

  SparkBaseConfig coralWristConfig;

  TalonSRX algeaIntake = new TalonSRX(0);

    @AutoLog
  public static class IntakeIOInputs {
    public double coralWristCurrent = 0.0;
    public double coralWristVelocity = 0.0;
    public double coralWristPosition = 0.0;
  }



  /** Creates a new IntakeIO. */
  public IntakeIO() {
    coralIntake = new TalonSRX(0);
    coralWrist = new SparkMax(0, MotorType.kBrushless);

    algeaIntake = new TalonSRX(0);

    algeaIntake.configFactoryDefault();
    algeaIntake.setNeutralMode(NeutralMode.Brake);
    coralIntake.configFactoryDefault();
    coralIntake.setNeutralMode(NeutralMode.Brake);
    
    coralWristConfig.closedLoop.pid(0, 0, 0);
    coralWristConfig.idleMode(IdleMode.kBrake);
    coralWrist.configure(coralWristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


  }


  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
  }

  public void setAlgaeVoltage(double voltage) {
    algeaIntake.set(TalonSRXControlMode.Current, voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.set(TalonSRXControlMode.Current, voltage);
  }

  public void adjustAngle(double angleRadians) {
    coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
  }

  public void wristAngle(double position) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public double getWristPosition() {
    return coralWrist.getEncoder().getPosition();
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
