// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.*;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Ports.*;

import frc.robot.Constants.*;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
import frc.robot.Constants;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax intakeMotor;
  private final PIDController pidController;
  private final DigitalInput beamBreak;
  private final SparkMax climbMotor;
  private final SparkMaxConfig climbConfig;
  private final SparkMaxConfig intakeConfig;

  public Intake() {
    intakeMotor = new SparkMax(IntakePorts.INTAKE_MOTOR, MotorType.kBrushless);
    pidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    beamBreak = new DigitalInput(IntakePorts.BEAM_BREAK);
    climbMotor = new SparkMax(Ports.ClimbPorts.FOLLOWER_MOTOR, MotorType.kBrushless);
    climbConfig = new SparkMaxConfig();
    intakeConfig = new SparkMaxConfig();
    climbConfig.follow(intakeMotor);
    climbConfig.inverted(true);
    intakeConfig.inverted(false);
    intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    climbMotor.configure(climbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  

  public Command runMotor() {
    return this.run(() -> intakeMotor.set(IntakeConstants.MOTOR_SPEED));
  }

  public Command reverseMotor() {
    return this.run(() -> intakeMotor.set(-IntakeConstants.MOTOR_SPEED));
  }

  public Command stopMotorCmd() {
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command setVoltage() {
    return this.run(() -> intakeMotor.setVoltage(IntakeConstants.VOLTAGE));
  }

  public boolean isBeamBroken() {
    return !beamBreak.get(); // true = broke, false = unbroken
  }

  public Command intakeCoral() {
    if (isBeamBroken())
      return this.run(() -> runMotor());
    else
      return this.run(() -> stopMotorCmd());
  }

  // public Command pulleySystemCmd() {
  //   return this.run(() -> climbMotor.set(ClimbConstants.CLIMB_SPEED));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
