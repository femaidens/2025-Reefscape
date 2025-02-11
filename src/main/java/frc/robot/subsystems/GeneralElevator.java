// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class GeneralElevator extends SubsystemBase{
  /** Creates a new GeneralElevator. */
  private ElevatorIO elevator = (Robot.isReal()) ? (new ActualElevator()) : (new Elevator());

  private PIDController elevatorPID = new PIDController(
      Constants.ElevatorConstants.PIDConstants.kP,
      Constants.ElevatorConstants.PIDConstants.kI,
      Constants.ElevatorConstants.PIDConstants.kD
    );
  
    private ElevatorFeedforward ff = new ElevatorFeedforward(
      Constants.ElevatorConstants.FeedForwardConstants.kS, 
      Constants.ElevatorConstants.FeedForwardConstants.kG, 
      Constants.ElevatorConstants.FeedForwardConstants.kV
    );
  // private ElevatorIO elevator = new NoElevator();
  
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSetpoint(double setpoint){
    elevatorPID.setSetpoint(setpoint);//*Math.sin(0.61) */
  }

  public Command reachGoal(double setpoint){
    this.setSetpoint(setpoint);
    if(Robot.isReal()){
      return this.run( () -> elevator.setVoltage(
        elevatorPID.calculate( elevator.getPosition() ) + 
        ff.calculate( elevatorPID.calculate(elevator.getPosition()) ) )); // not sure if this is correct
      // elevatorMotorFollower.resumeFollowerMode();
    }
    else{     
      return this.run( () -> elevator.setVoltage(
      elevatorPID.calculate( elevator.getPosition() ) + 
      ff.calculate( elevatorPID.calculate(elevator.getPosition()) ) ) );
    }
  }


}
