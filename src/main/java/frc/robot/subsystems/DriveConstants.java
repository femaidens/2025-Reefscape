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
        public static final double GEAR_RATIO = 6.75;
        public static final double DRIVE_MOTOR_FREE_SPEED = Units.feetToMeters(15);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final int DRIVE_MOTOR_PINION_TEETH = 14;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_REDUCTION;


        //modules stuff
        public static final int CURRENT_LIMIT = 35;
        public static final double POS_CONVERSION_FACTOR = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO; // meters;
        public static final double VEL_CONVERSION_FACTOR = ((WHEEL_DIAMETER * Math.PI) / GEAR_RATIO); // meters per second
        // public static final int AVERAGE_DEPTH = 0;

        public static final double FRONT_LEFT_ANGOFFSET = Math.PI; //-Math.PI / 2;
        public static final double FRONT_RIGHT_ANGOFFSET = 0;
        public static final double REAR_LEFT_ANGOFFSET = Math.PI; //Math.PI;
        public static final double REAR_RIGHT_ANGOFFSET = 0; //Math.PI / 2; 

        public static final double FRONT_LEFT_MAG_OFFSET = -0.4106;
        public static final double FRONT_RIGHT_MAG_OFFSET = -0.1301;
        public static final double REAR_LEFT_MAG_OFFSET = -0.1408;
        public static final double REAR_RIGHT_MAG_OFFSET = -0.5491;

        // CHANGE THIS FOR TALONS! 
        public static final String CANBUS = "rio"; 
        
        
        public class PID {
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 0.00005;
        }
        public class FF {
            public static final double S = .18484;
            public static final double V = 2.7542;
            public static final double A = 0.41189; 
        }
    }
    public class Turn {
        public static final int CURRENT_LIMIT = 35;
        public static final double POS_CONVERSION_FACTOR = 1.0 / (Math.PI * 2); //(2*Math.PI)/(150/7);
        public static final double VEL_CONVERSION_FACTOR = (2*Math.PI)/(150/7);
        public static final int AVERAGE_DEPTH = 0;

        public class PID {
            public static final double P = 2.5;
            public static final double I = 0;
            public static final double D = 0;   
        }
        
        public class FF {
            public static final double S = 0;
            public static final double V = 0;
            public static final double A = 0; 
            // public static final double G = 0;
        }
    }
    //**************DRIVETRAIN CONSTANTS******************//
    public class Drivetrain {
        public static final double TRACK_WIDTH = Units.inchesToMeters(28.9); // distance between right and left
        public static final double WHEEL_BASE = Units.inchesToMeters(28.9); // distance between front and back
        // VERIFY THIS IS IN THE CORRECT ORDER
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //fl
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //fr
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //rl
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) //rr
        );
        public static final double MAX_SPEED = 4.8; //in meters
        public static final double MAX_ROT_SPEED = Math.PI * 2; //in rad/s
        public static final double SPEED_FACTOR = 1.0;
    }
    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
      }
}