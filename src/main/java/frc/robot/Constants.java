// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kLeftJoystickDeadband = 0.1;
    public static final double kRightJoystickDeadband = 0.1;
  }

  public static class MotorIds {
    public static final int kIntakeMotor = 9;
    public static final int kAlgeaWristMotor = 10;

    public static final int kElevatorLeadMotor = 15;
    public static final int kElevatorFollowMotor = 16;

    public static final int kEndgameLeftMotor = 30;
    public static final int kEndgameRightMotor = 31;
  }

  public static class ElevatorHeights {
    public static final double kL1Height = 2.5;
    public static final double kL2Height = 4;
    public static final double kL3Height = 6;
    public static final double kL4Height = 9.23;
    public static final double kProcessorHeight = 8;
  }

  public enum ElevatorState {
    L1,
    L2,
    L3,
    L4,
    PROCESSOR,
    DOWN
  }

  public static class ElevatorTrapezoid {
    public static final double maxVelocity = 4.0;
    public static final double maxAcceleration = 2.0;
  }

  public static class WristTrapezoid {
    public static final double maxVelocity = 2;
    public static final double maxAcceleration = 1.5;
  }
}
