package frc.robot.unittesting;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TestingLib {
    public static void fastForward(int ticks) {
        for (int i = 0; i < ticks; i++) {
            CommandScheduler.getInstance().run();
            SimHooks.stepTiming(0.02);
        }
    }
    
}