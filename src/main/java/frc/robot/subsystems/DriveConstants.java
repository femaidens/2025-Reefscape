// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/** Add your docs here. */
public class DriveConstants {
    //**************MODULE CONSTANTS******************//
    public class Translation {
        public static final int CURRENT_LIMIT = 40;
        public static final double POS_CONVERSION_FACTOR = 0.0762 * Math.PI / ((45.0 * 22) / (14.0 * 15));
        public static final double VEL_CONVERSION_FACTOR = POS_CONVERSION_FACTOR;
        public static final int AVERAGE_DEPTH = 1;

        public static final double FRONT_LEFT_ANGOFFSET = -Math.PI/2;
        public static final double FRONT_RIGHT_ANGOFFSET = 0;
        public static final double REAR_LEFT_ANGOFFSET = Math.PI;
        public static final double REAR_RIGHT_ANGOFFSET = Math.PI/2; 
        
        public class PID {
            public static final double P = 0.2;
            public static final double I = 0;
            public static final double D = 0;
        }
        public class FF {
            public static final double S = 0.2;
            public static final double V = 0.5;
        }
    }
    public class Turn {
        public static final int CURRENT_LIMIT = 30;
        public static final double POS_CONVERSION_FACTOR = 2.0 * Math.PI;
        public static final double VEL_CONVERSION_FACTOR = 2.0 * Math.PI / 60.0;
        public static final int AVERAGE_DEPTH = 1;

        public class PID {
            public static final double P = 1;
            public static final double I = 0;
            public static final double D = 0;   
        }
        
        public class FF {
            public static final double S = 0;
            public static final double V = 0;
            public static final double G = 0;
        }
    }
    public class ModuleSimConstants{
    }

      public class DriveSimConstants{
        public static final double DRIVE_FORWARD_VOLTAGE = 5;
      }
    //**************DRIVETRAIN CONSTANTS******************//
    public class Drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(30); // distance between right and left
        public static final double WHEEL_BASE = Units.inchesToMeters(30); // distance between front and back
        // VERIFY THIS IS IN THE CORRECT ORDER
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
        );
        public static final double MAX_SPEED = 15; //in meters
        public static final double MAX_ROT_SPEED = 8; //in meters
        public static final double SPEED_FACTOR = 1.0;//Math.random() * Integer.MAX_VALUE;
    }
}