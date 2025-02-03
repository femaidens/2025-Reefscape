// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class Elevating {

    private Elevator elevator;

    public Elevating(){
        this.elevator = new Elevator();
    }
    /**
     * @return goes to first level of reef
     */
    public Command firstLevel(){
        return elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FIRST_LVL);
    }
    /**
     * 
     * @return goes to second level of reef
     */
    public Command secondLevel(){
        return elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.SECOND_LVL);
    }
    /**
     * 
     * @return goes to third level of reef
     */
    public Command thirdLevel(){
        return elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.THIRD_LVL);
    }
    /**
     * 
     * @return goes to the fourth level of reef
     */
    public Command fourthLevel(){
        return elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FOURTH_LVL);
    }
}
