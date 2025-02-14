// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Ports.*;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax intakeMotor;
  private final SparkMaxConfig config;
  private final DigitalInput beamBreak;
  private final PIDController intakePID;
  private final RelativeEncoder encoder;

  public Intake() {
    intakeMotor = new SparkMax(IntakePorts.INTAKE_MOTOR, MotorType.kBrushless);
    beamBreak = new DigitalInput(IntakePorts.BEAM_BREAK);
    config = new SparkMaxConfig();
    intakePID = new PIDController(IntakeConstants.IntakePIDConstants.kP, IntakeConstants.IntakePIDConstants.kI, IntakeConstants.IntakePIDConstants.kD);
    encoder = intakeMotor.getEncoder();

    // Configure the motor
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(IntakeConstants.VELOCITY_CONVERSION_FACTOR);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakePID(double setpoint){
    intakeMotor.set(intakePID.calculate(encoder.getVelocity(), setpoint));
  }
  
  public Command runMotorCmd() {
    return this.run(() -> intakeMotor.set(IntakeConstants.MOTORSPEED));
  }


  public Command reverseMotorCmd() {
    return this.run(() -> intakeMotor.set(-IntakeConstants.MOTORSPEED));

  }

  public Command stopMotorCmd() {
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> intakeMotor.setVoltage(voltage));
  }

  public boolean isBeamBroken() {
    return !beamBreak.get();
  }
  
  public Command intakeCoralCmd() {
    if (isBeamBroken()) {
      return this.run(() -> runMotorCmd());
    } else {
      return this.run(() -> stopMotorCmd());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Beambreak", isBeamBroken());
  }
}
