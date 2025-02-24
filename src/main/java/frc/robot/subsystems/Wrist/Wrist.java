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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.MotorIds;
import frc.robot.Constants.WristTrapezoid;
import frc.robot.commands.Wrist.WristControl;

public class Wrist extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFXConfiguration wristMotorConfig;

  public final double encoderOffset = 94.6;

  public double targetAngle = 106;

    // PID and Feedforward constants
  private static final double kP = 0.001;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = (0.55 - 0.4) / 2;   // Static
  private static final double kV = 0.0;   // Velocity
  private static final double kG = (0.55 + 0.4) / 2;  // Gravity

  private final double maxVelocity = WristTrapezoid.maxVelocity;
  private final double maxAcceleration = WristTrapezoid.maxAcceleration;

  private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

  public boolean isAtRest = true;
  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor = new TalonFX(MotorIds.kAlgeaWristMotor);
    wristMotorConfig = new TalonFXConfiguration();

    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    wristMotor.getConfigurator().apply(wristMotorConfig);

    pidController.setTolerance(0.5);
    targetAngle = getDegree();
    // setDefaultCommand(new WristControl(this, RobotContainer.m_operatorController));
  }

  public double moveToDegree(){
    double pidOutput = pidController.calculate(getDegree(), targetAngle);

    double feedforwardOutput = feedforward.calculate(Math.toRadians(getDegree()), pidOutput);

    return feedforwardOutput + pidOutput;
  }

  public double getDegree(){
    return ((wristMotor.getPosition().getValue().in(Degrees) / 11.38 ) - encoderOffset) * -1;
  }

  public void setWristVoltage(double voltage){
    wristMotor.setVoltage(-voltage);
    SmartDashboard.putNumber("Wrist voltage", voltage);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist raw encoder", wristMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber("Wrist Degree", getDegree());
    SmartDashboard.putNumber("Wrist pid output", pidController.calculate(getDegree(), targetAngle));
    SmartDashboard.putNumber("Wrist feedforward output", feedforward.calculate(Math.toRadians(getDegree()), pidController.calculate(getDegree(), targetAngle)));
  }
}
