
package frc.robot;

/** Add your docs here. */
public class Ports {
    public class OuttakePorts {
        public static int OUTTAKE_MOTOR = 0;


    }

    // public class UltrasonicPorts {
    //     public static int UltrasonicPingPort = 0;
    //     public static int UltrasonicEchoPort = 1;
      
    // }

    public class BeamBreakPorts { // for outtake 
        public static int FRONT_RECIEVER = 0; // front reciever is the one farthest away from intake
        public static int BACK_RECIEVER = 0;  // back reciever is the one closest to intake
    }
    public class IntakePorts{
        public static int INTAKE_MOTOR = 0;
        public static int BEAM_BREAK = 1;
    }
}

