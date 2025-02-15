// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class GeneralElevator extends SubsystemBase implements AutoCloseable{
  /** Creates a new GeneralElevator. */
  // private ElevatorIO elevator = (Robot.isReal()) ? (new ActualElevator()) : (new Elevator());
  
  private static DigitalInput botLimitSwitch = new DigitalInput( Ports.ElevatorPorts.BOT_SWITCH );


  private ElevatorIO hardware;

  public GeneralElevator(ElevatorIO hardware) {
    this.hardware = hardware;
  }

  public static GeneralElevator create() {
    return (Robot.isReal()) ? new GeneralElevator(new ActualElevator()) : new GeneralElevator(new Elevator());
  }

  public static GeneralElevator none() {
    return new GeneralElevator(new NoElevator());
  }


  private ProfiledPIDController elevatorPID = new ProfiledPIDController(
      Constants.ElevatorConstants.PIDConstants.kP,
      Constants.ElevatorConstants.PIDConstants.kI,
      Constants.ElevatorConstants.PIDConstants.kD,
      Constants.ElevatorConstants.PIDConstants.CONSTRAINTS
    );
  
    private ElevatorFeedforward ff = new ElevatorFeedforward(
      Constants.ElevatorConstants.FeedForwardConstants.kS, 
      Constants.ElevatorConstants.FeedForwardConstants.kG, 
      Constants.ElevatorConstants.FeedForwardConstants.kV
    );
  // private ElevatorIO elevator = new NoElevator();
  
  public void periodic() {
    // This method will be called once per scheduler run
    this.hitBotLimit();
  }
@Override 

  public void close(){
    this.close();
  }

  /**
     * if the limit switch is activated, the elevator motor stops moving
     */
    public void hitBotLimit(){
      if(botLimitSwitch.get()){
        elevator.setVoltage(0);
      }
    }

  public void setSetpoint(double setpoint){
    elevatorPID.setGoal(setpoint);//*Math.sin(0.61) */
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
      return this.run( () -> {
        double pidOutput = elevatorPID.calculate( elevator.getPosition()); 
        double ffOutput = ff.calculate( elevatorPID.getSetpoint().velocity);

        elevator.setVoltage(pidOutput + ffOutput);
      });
    }
  }
}