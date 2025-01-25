// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


/** Add your docs here. */
public class ElevatorCommands {
    private final Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command stage1Command() {
        // stage 1
        return elevator.reachGoalCommand(Constants.kSetpointMetersFirst);
    }

    public Command stage2Command() {
        // stage 2
        return elevator.reachGoalCommand(Constants.kSetpointMetersSecond);
    }

    public Command stage3Command() {
        // stage 3
        return elevator.reachGoalCommand(Constants.kSetpointMetersThird);
    }

    public Command stage4Command() {
        // stage 4
        return elevator.reachGoalCommand(Constants.kSetpointMetersFourth);
    }
}