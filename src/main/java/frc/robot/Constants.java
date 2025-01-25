// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final double kElevatorKp = 10;
  public static final double kElevatorKi = 0;
  public static final double kElevatorKd = 10;

  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.762; // volts (V)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

  public static final double kElevatorGearing = 10.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double kCarriageMass = 4.0; // kg

  public static final double kSetpointMetersFirst = 10;
  public static final double kSetpointMetersSecond = 12.5;
  public static final double kSetpointMetersThird = 15;
  public static final double kSetpointMetersFourth = 20;
  public static final double kStage2Velocity = 3;
  public static final double kStage3Velocity = 3;
  public static final double stage1Height = 10;
  public static final double stage2Height = 7;
  public static final double stage3Height = 5;
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double kMinElevatorHeightMeters = 0;
  public static final double kMaxElevatorHeightMeters = 10;

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  public static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;
}
