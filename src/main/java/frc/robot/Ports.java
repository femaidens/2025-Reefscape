package frc.robot;

public class Ports {
    public class DrivetrainPorts {
        public static final int FRONT_LEFT_DRIVE = 13;
        public static final int REAR_LEFT_DRIVE = 12;
        public static final int FRONT_RIGHT_DRIVE = 11;
        public static final int REAR_RIGHT_DRIVE = 10;
    
        public static final int FRONT_LEFT_TURN = 9;
        public static final int REAR_LEFT_TURN = 4;
        public static final int FRONT_RIGHT_TURN = 15;
        public static final int REAR_RIGHT_TURN = 20;

        public static final int FRONT_LEFT_CANCODER = 0;
        public static final int FRONT_RIGHT_CANCODER = 3;
        public static final int REAR_LEFT_CANCODER = 2;
        public static final int REAR_RIGHT_CANCODER = 1;
    }

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

    // public class ClimbPorts {
    //   public static int LEADER_PORT = 0; 
    //   public static int FOLLOWER_PORT = 0;
    // }
  
    public class AlgaeIntakePorts {
      public static final int INTAKE_ROLLER_LEADER = 3;
      public static final int INTAKE_ROLLER_FOLLOWER = 2;
    }
    
    public class AlgaePivotPorts {
      public static final int INTAKE_PIVOT_LEADER = 0;
      public static final int INTAKE_PIVOT_FOLLOWER = 0;
    }
}
