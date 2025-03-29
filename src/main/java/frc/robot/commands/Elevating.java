// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
//import frc.robot.subsystems.Intake;



/** Add your docs here. */
public class Elevating {

    public Elevator elevator;
    public Outtake outtake;
    //private Intake intake;

    public Elevating(Elevator elevator, Outtake outtake){
        this.elevator = elevator;
        this.outtake = outtake;
        //this.intake = intake;
    }

    /**
     * @return goes to first level of reef and then outtakes
     */
    public Command scoringAlgaeBargeCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.BARGE_LVL);
            // .andThen(outtake.setOuttakeCoralSpeedCmd());
    }
    
    public Command firstLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FIRST_LVL).withTimeout(2)
            .andThen(elevator.stopMotorCmd())
            .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

    /**
     * 
     * @return goes to second level of reef and then outtakes
     */
    public Command secondLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.SECOND_LVL);//.withTimeout(2)
            // .andThen(elevator.stopMotorCmd())
            // .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

    /**
     * 
     * @return goes to third level of reef and then outtakes
     */
    public Command thirdLevelCmd(){
        return
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.THIRD_LVL);//.withTimeout(2)
            // .andThen(elevator.stopMotorCmd())
            // .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

    /**
     * 
     * @return goes to the fourth level of reef and then outtakes
     */
    public Command fourthLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FOURTH_LVL);//.withTimeout(2)
            // .andThen(elevator.stopMotorCmd())
            // .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

        /**
     * 
     * @return possibly goes to the fourth level of reef and then outtakes and moves up
     */
    public Command possibleFourthLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.FOURTH_LVL)
            .until(elevator::atSetpoint)
            .andThen(outtake.runMotorCmd()).withTimeout(1.05)
            .andThen(elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.POSSIBLE_FOURTH_LVL))
            .alongWith(outtake.runMotorCmd());
            // .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

    /**
     * @return goes to algae removal level 2 and then removes
     */
    public Command algaeSecondLevelCmd(){
        return 
            elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.ALGAE_SECOND_LVL);//.withTimeout(2)
            // .andThen(elevator.stopMotorCmd())
            // .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

    /**
     * @return goes to algae removal level 3 and removes
     * someone else should do this one!!!
     */
    public Command algaeThirdLevelCmd(){
        return 
        elevator.setLevel(Constants.ElevatorConstants.SetpointConstants.ALGAE_THIRD_LVL).withTimeout(2)
        .andThen(elevator.stopMotorCmd())
        .andThen(outtake.setOuttakeCoralSpeedCmd()).withTimeout(2);
    }

    /**
     * @return resets everything to default
     */
    public Command resetDefault(){
        return 
            elevator.setLevelWithLimit(Constants.ElevatorConstants.SetpointConstants.DEFAULT_LVL);
            // .andThen(elevator.stopMotorCmd())
            // .andThen(outtake.stopMotorCmd());
        
            
    }
}
