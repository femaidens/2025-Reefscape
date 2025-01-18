// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

/** Add your docs here. */
public class OuttakingCoral {
    private final Outtake outtake; 

    public OuttakingCoral(Outtake outtake) {
       this.outtake = outtake;

    }

    public Command releaseCoral() {
        return outtake.reverseMotorCmd();
    }
}
