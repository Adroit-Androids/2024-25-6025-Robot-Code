// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class telescopicArm extends SubsystemBase {
  double Kp;
  double ki;
  double Kd;
  double maxVelocity;
  double maxAcceleration;




  private final TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

  ProfiledPIDController armController = new ProfiledPIDController(Kp,ki, Kd, armConstraints);


  WPI_TalonSRX telescopicArm = new WPI_TalonSRX(0);

  /** Creates a new telescopicArm. */
  public telescopicArm() {
    telescopicArm.configFactoryDefault();
    telescopicArm.setNeutralMode(NeutralMode.Brake);
  }


  public double encoderValue(){
    return telescopicArm.getSelectedSensorPosition();
  }

  public void resetEncoder(){
    telescopicArm.setSelectedSensorPosition(0);
  }

  public void setSpeed(double speed){
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
