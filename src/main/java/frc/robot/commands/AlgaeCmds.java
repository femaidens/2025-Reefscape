// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
/** Add your docs here. */
public class AlgaeCmds {
    private AlgaeIntake algaeIntake;
    private AlgaePivot algaePivot;
    public AlgaeCmds(AlgaeIntake algaeIntake, AlgaePivot algaePivot){
        this.algaeIntake = algaeIntake;
        this.algaePivot = algaePivot;
    }

    public Command intakeAlgae(){
        return 
            algaePivot.setGroundCmd()
            .alongWith(algaeIntake.runRollersCmd());
    }

    public Command raiseAlgae(){
        return
            algaePivot.setProcessorCmd();
    }

    public Command outtakeAlgae(){
        return
            algaeIntake.reverseRollersCmd();
    }
}
