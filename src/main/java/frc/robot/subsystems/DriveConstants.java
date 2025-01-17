// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    //**************MODULE CONSTANTS******************//
    public class Translation {
        public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER = 0.0762; // meters
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final int DRIVE_MOTOR_PINION_TEETH = 14;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_REDUCTION;


        //modules stuff
        public static final int CURRENT_LIMIT = 35;
        public static final double POS_CONVERSION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION; // meters;
        public static final double VEL_CONVERSION_FACTOR = ((WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second
        // public static final int AVERAGE_DEPTH = 0;

        public static final double FRONT_LEFT_ANGOFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_ANGOFFSET = 0;
        public static final double REAR_LEFT_ANGOFFSET = Math.PI;
        public static final double REAR_RIGHT_ANGOFFSET = Math.PI / 2; 
        
        public class PID {
            public static final double P = 0.1;
            public static final double I = 0;
            public static final double D = 0;
        }
        public class FF {
            public static final double S = 0;
            public static final double V = 0;
        }
    }
    public class Turn {
        public static final int CURRENT_LIMIT = 35;
        public static final double POS_CONVERSION_FACTOR = 0;
        public static final double VEL_CONVERSION_FACTOR = 0;
        public static final int AVERAGE_DEPTH = 0;

        public class PID {
            public static final double P = 0.1;
            public static final double I = 0;
            public static final double D = 0;   
        }
        
        public class FF {
            public static final double S = 0;
            public static final double V = 0;
            // public static final double G = 0;
        }
    }
    //**************DRIVETRAIN CONSTANTS******************//
    public class Drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(26); // distance between right and left
        public static final double WHEEL_BASE = Units.inchesToMeters(26); // distance between front and back
        // VERIFY THIS IS IN THE CORRECT ORDER
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
        );
        public static final double MAX_SPEED = 4.8; //in meters
        public static final double MAX_ROT_SPEED = Math.PI * 2; //in rad/s
        public static final double SPEED_FACTOR = 1.0;
    }
    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
      }
}