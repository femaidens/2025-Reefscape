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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.ElevatorPIDConstants;
import frc.robot.Ports.ArmPIDPorts;
import frc.robot.Ports.ElevatorPIDPorts;

public class ElevatorPID extends SubsystemBase {
  private static SparkMax rightMotor;
  private static SparkMax leftMotor;
  private static RelativeEncoder relativeEncoder;
  private static AbsoluteEncoder absoluteEncoder;
  private static PIDController reverseElevatorPID;
  private static PIDController elevatorPID;
  

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

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorPIDConstants.CURRENT_LIMIT)
        .follow(rightMotor, true);

        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
