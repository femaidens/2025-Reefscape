// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
/** Add your docs here. */
public class AlgaeCmds {
    private AlgaeIntake algaeIntake;
    public AlgaeCmds(){
        algaeIntake = new AlgaeIntake();
    }

    public Command intakeAlgae(){
        return 
            algaeIntake.setGround()
            .andThen(algaeIntake.intakeAlgae());
    }

    // public Command raiseAlgae(){
    //     return
    //         algaeIntake.setProcessor();
    // }

    public Command outtakeAlgae(){
        return
            algaeIntake.runRollers();
    }
}
