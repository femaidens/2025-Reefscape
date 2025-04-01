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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PathPlannerConstants {
    public static final double massKg = Units.lbsToKilograms(120);
    public static final double MOI = 1.0/12 * massKg * (Math.pow(DriveConstants.Drivetrain.TRACK_WIDTH, 2) + Math.pow(DriveConstants.Drivetrain.WHEEL_BASE, 2));
    // double WHEEL_RADIUS;
    // double MAX_DRIVE_VELOCITY_MPS;
    // double WHEEL_COF;
    // double DRIVE_GEARING;
    // double DRIVE_CURRENT_LIMIT;
    
  }
  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Transform2d robotToCam = new Transform2d(
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation2d(Units.degreesToRadians(0), Units.degreesToRadians(0))); //only for the new drive to pose cmd that uses tag data
    public static final Transform3d kFrontLeftCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
    public static final Transform3d kFrontRightCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
    public static final Transform3d kRearLeftCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
    public static final Transform3d kRearRightCamToCenter = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));

    //for a strange aligning to target manually idea
    //i think its a percentage?
    public static final double GOAL_AREA_RIGHT = 9;
    public static final double GOAL_X_RIGHT = 110; //106; //225.75; // x position of apriltag crosshair
    public static final double GOAL_AREA_LEFT = 9;
    public static final double GOAL_X_LEFT = 200; //195 //216; //225.75; // x position of apriltag crosshair
    public static final double GOAL_X_MIDDLE = 250;
    public static final double GOAL_AREA_MIDDLE = 9;
    // public static final double GOAL_Y = 150; //220.25; // y position of apriltag crosshair
    
    public class PID {
      public static final double P = 4.0;
      public static final double I = 0;
      public static final double D = 0;
    
      
    }
  }
  public static class ElevatorConstants {
    public static final int CURRENT_LIMIT = 40;
    public static final double MOTOR_SPEED = 0.65;
    public static final double REVERSE_MOTOR_SPEED = 0.3;
    public static final double FORCE_MOTOR_SPEED = 0.1;
    public static final double GEAR_RATIO = 1.0 / 20.0;
    public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
    public static final double ABSOLUTE_OFFSET = 0.3;

    public static class PIDConstants {
      public static final double kP = 7;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kMaxVelocity = 7;
      public static final double kMaxAcceleration = 10;
      public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(kMaxVelocity,
          kMaxAcceleration);
    }

    public static class FeedForwardConstants {
      public static final double kS = 0.25722;
      public static final double kG = 0.18831;
      public static final double kV = 7.6552;
      public static final double kA = 0.56958;
    }

    public static class SetpointConstants {
      public static final double DEFAULT_LVL = 0;
      public static final double FIRST_LVL = 1.3; //2.5;
      public static final double SECOND_LVL = 1.6; //3.59;
      public static final double ALGAE_SECOND_LVL = 1.42;
      public static final double THIRD_LVL = 3.4; //6.28;
      public static final double ALGAE_THIRD_LVL = 0;
      public static final double FOURTH_LVL = 6.5; 
      // public static final double DEFAULT_LVL = 1.54; //1.83;
      public static final double MINIMUM_LVL = 0;
      public static final double MAXIMUM_LVL = 7.8;
      public static final double BARGE_LVL = 7.4;

      public static final double POSSIBLE_FOURTH_LVL  = 7.2;
    }
  }
  public static class IntakeConstants {
    public static final double MOTORSPEED = 0.5;
    public static final double VOLTAGE = 0.0;
    public static final double VELOCITY_CONVERSION_FACTOR = 1000.0;
    public static final double POSITION_CONVERSION_FACTOR = 1000.0;
    public static final int CURRENT_LIMIT = 30;

    public static class IntakePIDConstants {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }
  public static class OuttakeConstants {
    public static final double MOTOR_SPEED = 0.3;
    public static final double BARGE_MOTOR_SPEED = 0.7;
    public static final double REMOVE_ALGAE_SPEED = 0.4; // some random value for now
    public static final double VOLTAGE = 0;
    public static final int CURRENT_LIMIT = 30;
  }
}
