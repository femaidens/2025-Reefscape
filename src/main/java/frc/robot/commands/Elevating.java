// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;


/** Add your docs here. */
public class Elevating {

    private Elevator elevator;
    private Outtake outtake;
    private Intake intake;
    

    public Elevating(){
        this.elevator = new Elevator();
        this.outtake = new Outtake();
        this.intake = new Intake();
    }

    /**
     * @return goes to first level of reef and then outtakes
     */
    public Command firstLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FIRST_LVL)
            .andThen(outtake.setOuttakeCoralSpeedCmd());
    }

    /**
     * 
     * @return goes to second level of reef and then outtakes
     */
    public Command secondLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.SECOND_LVL)
            .andThen(outtake.setOuttakeCoralSpeedCmd());
    }

    /**
     * 
     * @return goes to third level of reef and then outtakes
     */
    public Command thirdLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.THIRD_LVL)
            .andThen(outtake.setOuttakeCoralSpeedCmd());
    }

    /**
     * 
     * @return goes to the fourth level of reef and then outtakes
     */
    public Command fourthLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FOURTH_LVL)
            .andThen(outtake.setOuttakeCoralSpeedCmd());
    }

    /**
     * @return goes to algae removal level 2 and then removes
     */
    public Command algaeSecondLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.ALGAE_SECOND_LVL)
            .andThen(outtake.setOuttakeCoralSpeedCmd());
    }

    /**
     * @return goes to algae removal level 3 and removes
     * someone else should do this one!!!
     */
    public Command algaeThirdLevelCmd(){
        return 
        elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.ALGAE_THIRD_LVL)
        .andThen(outtake.setOuttakeCoralSpeedCmd());
    }

    /**
     * @return resets everything to default
     */
    public Command resetDefault(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FIRST_LVL)
            .andThen(elevator.stopMotorCmd())
            .alongWith(intake.stopMotorCmd())
            .alongWith(outtake.stopMotorCmd());
    }
}
