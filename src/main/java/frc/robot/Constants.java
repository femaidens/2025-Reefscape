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
  }


public static class ClimbConstants {
  public static final double CLIMB_SPEED = 0.5;
  public static final double TICKS_PER_REVOLUTION = 1000;
  public static final double MAX_ROTATION = 90.0;
  public static final double MIN_ROTATION = -90.0; //subject to change, need to change to rotation??

}

public static class PIDConstants {
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kMaxVelocity = 0;
  public static final double kMaxAcceleration = 0;
  //public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
}

public static class IntakeConstants {
  public static final double MOTORSPEED = 0.5;
  public static final double VOLTAGE = 0.0;
  public static final double VELOCITY_CONVERSION_FACTOR = 1000.0;
  public static final double POSITION_CONVERSION_FACTOR = 1000.0;
  public static final int CURRENT_LIMIT = 30;
}

public static class IntakePivotConstants {
  public static final double PIVOT_SPEED = 0.4;
  public static final double MOTOR_SPEED = 0.5;
  public static final double VOLTAGE = 0;
}

public static class FeedForwardConstants {
  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0;
}
}
