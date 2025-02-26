// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class Wrist extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFXConfiguration wristMotorConfig;

  public final double encoderOffset = 104;

  public double targetAngle = 90;

    // PID and Feedforward constants
  private static final double kP = 0.04;
  private static final double kI = 0.0;
  private static final double kD = 0.003;
  private static final double kS = (0.43 - 0.41) / 2;   // Static
  private static final double kV = 0.0;   // Velocity
  private static final double kG = (0.43 + 0.41) / 2;  // Gravity


  private final PIDController pidController = new PIDController(kP, kI, kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG * 1.1, kV);

  public boolean isAtRest = true;
  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor = new TalonFX(MotorIds.kAlgeaWristMotor);
    wristMotorConfig = new TalonFXConfiguration();

    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    wristMotor.getConfigurator().apply(wristMotorConfig);

    wristMotor.setPosition(0);
    pidController.setTolerance(3);
    //targetAngle = getDegree();
    //setDefaultCommand(new WristControl(this, RobotContainer.m_operatorController));
  }

  public double moveToDegree(){
    double pidOutput = pidController.calculate(getDegree(), targetAngle);

    double feedforwardOutput = feedforward.calculate(Math.toRadians(getDegree()), pidOutput);

    return feedforwardOutput + pidOutput;
  }

  public double getDegree(){
    return ((wristMotor.getPosition().getValue().in(Degrees) / 11.53 ) - encoderOffset) * -1;
  }

  public void setWristVoltage(double voltage){
    if (Math.abs(voltage) < 3){
      wristMotor.setVoltage(-voltage);
    }
    else {
      wristMotor.setVoltage(-3 * Math.signum(voltage));
    }

    SmartDashboard.putNumber("Wrist voltage", voltage);
  }

  @Override
  public void periodic() {
    // if (!pidController.atSetpoint()){
    //   setWristVoltage(moveToDegree());
    // }
    // else {
    //   setWristVoltage(feedforward.calculate(Math.toRadians(getDegree()), 1));
    // }
    setWristVoltage(moveToDegree());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist raw encoder", wristMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber("Wrist target", targetAngle);
    SmartDashboard.putNumber("Wrist Degree", getDegree());
    SmartDashboard.putNumber("Wrist pid output", pidController.calculate(getDegree(), targetAngle));
    SmartDashboard.putNumber("Wrist feedforward output", feedforward.calculate(Math.toRadians(getDegree()), 1));
  }
}
