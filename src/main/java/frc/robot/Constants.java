// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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
  public static class VisionConstants {
    public static final String cameraName = "arduCam1";

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(0.5, 0.0, 0.5),
      new Rotation3d(0, 0, 0));
  }
  public static final class PID {
    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final class FF {
    public static final double kS = 0.01;
    public static final double kV = 0.01;
  }

  public static final class DriveConstants {
    public static final double MAX_SPEED = 0.2; // Meters per second
    public static final double CIRCUMFERENCE = 0.1;

    // public static final double GEAR_RATIO = 6.75;
    // public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    // public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double POSITION_FACTOR = 0;
    public static final double VELOCITY_FACTOR = 0;
  }

}
