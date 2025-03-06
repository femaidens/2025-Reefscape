package frc.robot;

public class Ports {
    public class DrivetrainPorts {
        public static final int FRONT_LEFT_DRIVE = 13;
        public static final int REAR_LEFT_DRIVE = 11;
        public static final int FRONT_RIGHT_DRIVE = 12;
        public static final int REAR_RIGHT_DRIVE = 10;
    
        public static final int FRONT_LEFT_TURN = 9;
        public static final int REAR_LEFT_TURN = 15;
        public static final int FRONT_RIGHT_TURN = 4;
        public static final int REAR_RIGHT_TURN = 20;

        public static final int FRONT_LEFT_CANCODER = 0;
        public static final int FRONT_RIGHT_CANCODER = 2;
        public static final int REAR_LEFT_CANCODER = 3;
        public static final int REAR_RIGHT_CANCODER = 1;
    }
    public class ElevatorPorts {
        public static int MOTOR_PORT = 15;
        // public static int BOT_SWITCH = 22;
    }
    public class IntakePorts {
        public static int INTAKE_MOTOR = 3;
        public static int BEAM_BREAK = 9;//8,9
        public static int PIVOT_MOTOR = 29;
      }
      public class OuttakePorts {
        public static int OUTTAKE_MOTOR =12 ; 
        public static int FRONT_RECEIVER = 0; // front reciever is the one farthest away from intake
        public static int BACK_RECEIVER = 1;
      }
      public class BeamBreakPorts { // for outtake
         // back reciever is the one closest to
        // intake
      }
}