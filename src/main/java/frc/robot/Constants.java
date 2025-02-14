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
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

    public static class AlgaeIntakeConstants {
      public static double POSITIONCONVERSIONFACTOR = 1000;
      public static double VELOCITYCONVERSIONFACTOR = 1000;
      public static double PIVOTVOLTAGE = 10;

      public static double PIVOTSETPOINT = 0;

      public class PIDConstants {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static double MAXVELOCITY = 5;
        public static double MAXACCELERATION = 10;
      }

      public class FFConstants {
        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;
      }
    }
}
