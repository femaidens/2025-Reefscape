// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

//import com.ctre.phoenix6.hardware.TalonFX;


public class ActualElevator extends SubsystemBase implements ElevatorIO {
  /** Creates a new Elevator. */

  private static SparkMax elevatorMotorLeader;
  private static SparkMax elevatorMotorFollower;
  private static RelativeEncoder elevatorEncoder;
   
  public ActualElevator() {
    elevatorMotorLeader = new SparkMax( Ports.ElevatorPorts.MOTOR_PORT, SparkLowLevel.MotorType.kBrushless );
    elevatorMotorFollower = new SparkMax( Ports.ElevatorPorts.MOTOR_PORT, SparkLowLevel.MotorType.kBrushless );

    elevatorEncoder = elevatorMotorLeader.getEncoder();

      SparkMaxConfig LeaderConfig = new SparkMaxConfig();

        LeaderConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);

        LeaderConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(Constants.ElevatorConstants.PIDConstants.kP, Constants.ElevatorConstants.PIDConstants.kI, Constants.ElevatorConstants.PIDConstants.kD);
        //probably unneeded 

        elevatorMotorLeader.configure(LeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(LeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        SparkMaxConfig FollowerConfig = new SparkMaxConfig();

        FollowerConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        FollowerConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(Constants.ElevatorConstants.PIDConstants.kP, Constants.ElevatorConstants.PIDConstants.kI, Constants.ElevatorConstants.PIDConstants.kD);
        //probably unneeded 

        elevatorMotorLeader.configure(LeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      
    }    

    


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
  }
  @Override
  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  // @Override
  // public double getVoltage() {
  //   return elevatorMotorLeader.getVoltage();
  // }

  @Override
  public void setVoltage(double voltage ) {
    elevatorMotorLeader.setVoltage(voltage);
  }

  @Override
  public void close() throws Exception {
    elevatorMotorFollower.close();
    elevatorMotorLeader.close();
  }
}
