// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

//import com.ctre.phoenix6.hardware.TalonFX;



public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private static SparkMax elevatorMotorLeader;
  private static SparkMax elevatorMotorFollower;
  private static DigitalInput botLimitSwitch;
  private static ProfiledPIDController elevatorPID;
  private static RelativeEncoder elevatorEncoder;
  private static ElevatorFeedforward ff;
   
  public Elevator() {
    elevatorMotorLeader = new SparkMax(Ports.ElevatorPorts.MOTOR_PORT, SparkLowLevel.MotorType.kBrushless);
    elevatorMotorFollower = new SparkMax(Ports.ElevatorPorts.MOTOR_PORT, SparkLowLevel.MotorType.kBrushless);

    botLimitSwitch = new DigitalInput( Ports.ElevatorPorts.BOT_SWITCH);

    elevatorEncoder = elevatorMotorLeader.getEncoder();

    elevatorPID = new ProfiledPIDController(
      Constants.ElevatorConstants.PIDConstants.kP, 
      Constants.ElevatorConstants.PIDConstants.kI, 
      Constants.ElevatorConstants.PIDConstants.kD, 
      Constants.ElevatorConstants.PIDConstants.CONSTRAINTS
      );

      ff = new ElevatorFeedforward(
        Constants.ElevatorConstants.FeedForwardConstants.kS, 
        Constants.ElevatorConstants.FeedForwardConstants.kG, 
        Constants.ElevatorConstants.FeedForwardConstants.kV
      );

      SparkMaxConfig config = new SparkMaxConfig();

        config
        .inverted(true)
        .idleMode(IdleMode.kBrake);

        config.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(Constants.ElevatorConstants.PIDConstants.kP, Constants.ElevatorConstants.PIDConstants.kI, Constants.ElevatorConstants.PIDConstants.kD);
        //probably unneeded 
        config
        .follow(elevatorMotorLeader, false);

        elevatorMotorLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /**
     * run the motor
     */
    public static void runMotor(){
      elevatorMotorLeader.set(Constants.ElevatorConstants.MOTOR_SPEED);
      elevatorMotorFollower.resumeFollowerMode();
      System.out.println(elevatorMotorFollower.isFollower());
    }

    /**
     * reverse motor
     */
    public static void reverseRunMotor(){
      
      elevatorMotorLeader.set(-Constants.ElevatorConstants.MOTOR_SPEED);
      elevatorMotorFollower.resumeFollowerMode();

    }
    /**
     * stops the motor from running
     */
    public static void stopMotor(){
      elevatorMotorLeader.set(0);
      elevatorMotorFollower.resumeFollowerMode();

    }
  
    /**
     * sets motor voltage using calculations from PID and FF values 
     */
    public static void elevatorPID(double setpoint){
      elevatorMotorLeader.setVoltage(
        elevatorPID.calculate( elevatorEncoder.getPosition() ) +  ff.calculate(elevatorPID.getSetpoint().velocity, setpoint));
        elevatorMotorFollower.resumeFollowerMode();
    }

    /**
     * if the limit switch is activated, the elevator motor stops moving
     */
    public void hitBotLimit(){
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
