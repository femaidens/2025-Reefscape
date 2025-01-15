// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
/**
 * limit switch
 * motor ( 1 )
 * PID - setpoint cmds
 * 
 */

  private static SparkMax elevatorMotor;
  private static DigitalInput botLimitSwitch;
  private static ProfiledPIDController elevatorPID;
  private static CANcoder elevatorEncoder;
  private static ElevatorFeedforward ff;
   
    public Elevator() {
  
      elevatorMotor = new SparkMax( Ports.ElevatorPorts.MOTOR_PORT, SparkLowLevel.MotorType.kBrushless);
  
      botLimitSwitch = new DigitalInput( Ports.ElevatorPorts.BOT_SWITCH);
  
      elevatorEncoder = new CANcoder( Ports.ElevatorPorts.ENCODER_PORT );
  
      elevatorPID = new ProfiledPIDController(
        Constants.ElevatorConstants.PIDConstants.kP, 
        Constants.ElevatorConstants.PIDConstants.kI, 
        Constants.ElevatorConstants.PIDConstants.kD, 
        Constants.ElevatorConstants.PIDConstants.constraints
        );
  
        ff = new ElevatorFeedforward(
          Constants.ElevatorConstants.FeedForwardConstants.kS, 
          Constants.ElevatorConstants.FeedForwardConstants.kG, 
          Constants.ElevatorConstants.FeedForwardConstants.kV
        );
    }
    /**
     * run the motor
     */
    public static void runMotor(){
      elevatorMotor.set(Constants.ElevatorConstants.motorSpeed);
    }

    /**
     * reverse motor
     */
    public static void reverseRunMotor(){
      
      elevatorMotor.set(-Constants.ElevatorConstants.motorSpeed);
    }
    /**
     * stops the motor from running
     */
    public static void stopMotor(){
      elevatorMotor.set(0);
    }
  
    /**
     * sets motor voltage using calculations from PID and FF values 
     */
    public static void elevatorPID(double setpoint){
      elevatorMotor.setVoltage(
        elevatorPID.calculate( elevatorEncoder.getPosition().getValueAsDouble() ) +  ff.calculate(elevatorPID.getSetpoint().velocity, setpoint));
    }

    /**
     * if the limit switch is activated, the elevator motor stops moving
     */
    public static void hitBotLimit(){
      if(botLimitSwitch.get()){
        stopMotor();
      }
    }

    //Cmds
    /**
     * @return run elevator motor command
     */
    public Command runMotorCmd() {
      return this.run(() -> runMotor());
    }

    /**
     * @return run reverse elevator motor command
     */
    public Command reverseRunMotorCmd() {
      return this.run(() -> reverseRunMotor());
    }

    /**
     * @return stop elevator motor command
     */
    public Command stopMotorCmd() {
      return this.run(() -> stopMotor());
    }

    /**
     * 
     * @param setpoint
     * @return lifts elevator to specified setpoint
     */
    public Command setLevel(double setpoint) {
      return this.run(() -> elevatorPID(setpoint));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.hitBotLimit();
    
  }
}
