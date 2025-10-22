// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Ports.ArmPIDPorts;
import frc.robot.Constants;
import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPID extends SubsystemBase {
  private static SparkMax topMotor;
  private static SparkMax middleMotor;
  private static SparkMax bottomMotor;
  private static AbsoluteEncoder armEncoder;
  private static AbsoluteEncoderConfig armEncoderConfig;
  private static PIDController armPID;
  private static double lastSetpoint;
  

  /** Creates a new ArmPID. */
  public ArmPID() {
    topMotor = new SparkMax(ArmPIDPorts.TOP_MOTOR, SparkLowLevel.MotorType.kBrushless);
    middleMotor = new SparkMax(ArmPIDPorts.MIDDLE_MOTOR, SparkLowLevel.MotorType.kBrushless);
    bottomMotor = new SparkMax(ArmPIDPorts.BOTTOM_MOTOR, SparkLowLevel.MotorType.kBrushless);
    armEncoder = topMotor.getAbsoluteEncoder();
    armEncoderConfig = new AbsoluteEncoderConfig();
    armEncoderConfig.positionConversionFactor(360);
    
    armPID = new PIDController(ArmPIDConstants.ArmPIDPIDConstants.kP, ArmPIDConstants.ArmPIDPIDConstants.kI,
        ArmPIDConstants.ArmPIDPIDConstants.kD);

    SparkMaxConfig topConfig = new SparkMaxConfig();
    topConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmPIDConstants.CURRENT_LIMIT);

    SparkMaxConfig middleConfig = new SparkMaxConfig();
    middleConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmPIDConstants.CURRENT_LIMIT)
        .follow(topMotor, true);

    SparkMaxConfig bottomConfig = new SparkMaxConfig();
    bottomConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmPIDConstants.CURRENT_LIMIT)
        .follow(topMotor, true);

   topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  middleMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setVelocityCmd(double setpoint){
    return this.run(() -> setVelocity(setpoint));
  }

  public Command setMotorSpeedCmd() {
    return this.runOnce(() -> {
      topMotor.set(ArmPIDConstants.MOTOR_SPEED);
      // middleMotor.set(ArmPIDConstants.MOTOR_SPEED);
      // bottomMotor.set(ArmPIDConstants.MOTOR_SPEED);
    });
  }

  public Command stopMotorCmd() {
    return this.runOnce(() -> {
      topMotor.set(0);
    });
  }

  public Command setCurrentSetpointCmd(double setpoint) {
    return this.run(() -> lastSetpoint = setpoint);
  }


  public void setVelocity(double setpoint){
    topMotor.setVoltage(armPID.calculate(getAngle(), setpoint));
  }

  public double getAngle(){
    return armEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle: ", getAngle());
  }
}
