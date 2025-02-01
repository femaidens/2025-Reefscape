// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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
  public static class ElevatorConstants {
    public static final double motorSpeed = 0;
    public static final double POSITIONCONVERSIONFACTOR = 1000;
    public static final double VELOCITYCONVERSIONFACTOR = 1000;


    public static class PIDConstants {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kMaxVelocity = 0;
      public static final double kMaxAcceleration = 0;
      public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    
    }

    public static class FeedForwardConstants {
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
    }

    public static class SetpointConstants {
      public static final double firstLvl = 0;
      public static final double secondLvl = 0;
      public static final double thirdLvl = 0;
      public static final double fourthLvl = 0;
    }
  }
}
