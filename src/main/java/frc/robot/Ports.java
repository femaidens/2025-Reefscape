// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Ports {
    public class ElevatorPorts{
        public static int MOTOR_PORT = 3;
        public static int BOT_SWITCH = 2;
    }

    public class OuttakePorts {
        public static int OUTTAKE_MOTOR = 0;
    }

    public class BeamBreakPorts { // for outtake 
        public static int FRONT_RECEIVER = 0; // front reciever is the one farthest away from intake
        public static int BACK_RECEIVER = 0;  // back reciever is the one closest to intake
    }
  
    public class IntakePorts{
        public static int INTAKE_MOTOR = 0;
        public static int BEAM_BREAK = 1;
        public static int PIVOT_MOTOR = 2;
    }

    public class ClimbPorts {
      public static final int LEADER_MOTOR = 0; 
      public static final int FOLLOWER_MOTOR = 0;
    }
  }