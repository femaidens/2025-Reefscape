// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.RelativeEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorPIDConstants;
import frc.robot.Ports.ElevatorPIDPorts;

public class ElevatorPID extends SubsystemBase {
  private static SparkMax rightMotor;
  private static SparkMax leftMotor;
  private static RelativeEncoder relativeEncoder;
  private static AbsoluteEncoder absoluteEncoder;
  private static PIDController reverseElevatorPID;
  private static PIDController elevatorPID;

  private static double lastSetpoint;
  

  /** Creates a new ElevatorPID. */
  public ElevatorPID() {
    rightMotor = new SparkMax(ElevatorPIDPorts.RIGHT_MOTOR, SparkLowLevel.MotorType.kBrushless);
    leftMotor = new SparkMax(ElevatorPIDPorts.LEFT_MOTOR, SparkLowLevel.MotorType.kBrushless);
    relativeEncoder = rightMotor.getEncoder();
    absoluteEncoder = rightMotor.getAbsoluteEncoder();


     elevatorPID = new PIDController(ElevatorPIDConstants.ElevatorPIDPIDConstants.kP, ElevatorPIDConstants.ElevatorPIDPIDConstants.kI,
        ElevatorPIDConstants.ElevatorPIDPIDConstants.kD);
      reverseElevatorPID = new PIDController(ElevatorPIDConstants.ReversePIDConstants.kP, ElevatorPIDConstants.ReversePIDConstants.kI,
        ElevatorPIDConstants.ReversePIDConstants.kD);

    
      SparkMaxConfig rightConfig = new SparkMaxConfig(); //right moter is the leader
    rightConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorPIDConstants.CURRENT_LIMIT);

    rightConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorPIDConstants.CURRENT_LIMIT)
        .follow(rightMotor, true);

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

        lastSetpoint = relativeEncoder.getPosition();
  }

  public Command runElevatorMotorCmd(){
    return this.run(()-> rightMotor.set(ElevatorPIDConstants.MOTOR_SPEED));
  }

  public Command stopElevatorMotorCmd(){
    return this.run(()-> rightMotor.set(0));
  }

 public Command reverseMotorCmd() {
  return this.run(()-> rightMotor.set(-ElevatorPIDConstants.MOTOR_SPEED));
 } 
 
  public void elevatorPid(){
      rightMotor.setVoltage(elevatorPID.calculate(relativeEncoder.getPosition(), lastSetpoint));
  }

  public void reverseElevatorPID(double setpoint){
    rightMotor.setVoltage(reverseElevatorPID.calculate(relativeEncoder.getPosition(), setpoint));
  }

  public double getCurrentPosition(){
    return relativeEncoder.getPosition();
  }

  public Command setCurrentSetpointCmd (double setpoint){
   return this.run(()-> lastSetpoint = setpoint);
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
