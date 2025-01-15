// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class elevator extends SubsystemBase {
  private DigitalInput magneticSwitchUpper = new DigitalInput(0);
  private DigitalInput magneticSwitchLower = new DigitalInput(1);

  private final elevatorIO io;
  //private final elevatorIOInputsLogging inputs = new elevatorIOInputsLogging();

  // Constructor
  public elevator (elevatorIO io) {
    this.io = io;
  }


  // Method to adjust/set power for the elevator
  public void setVoltage(double voltage){
    System.out.println("Elevator position: " + getPosition());
    io.set(voltage);
  }

  
  // Method to stop the elevator
  public void stop(){
    io.stop();
  }


  //Method to set the elevator to a specific desired position
  public void setPosition(double position){
    io.setPosition(position);
  }



// ----------------------
  public double getPosition(){
    return io.getPosition();
  }

  public double getVelocity(){
    return io.getVelocity();
  }

  public void resetPosition(){
    io.resetPosition();
  }
// -----------------------



  @Override
  public void periodic() {
    if (magneticSwitchUpper.get() == true){
      io.stop();
    }
    if (magneticSwitchLower.get() == true){
      io.stop();
    }
    //io.updateInputs(inputs);
  }
}
