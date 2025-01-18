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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax intakeMotor;
  private final PIDController pidController;
  private final DigitalInput beamBreak;

  public Intake() {
    intakeMotor = new SparkMax(IntakePorts.INTAKE_MOTOR, MotorType.kBrushless);
    pidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    beamBreak = new DigitalInput(IntakePorts.BEAM_BREAK);
  }

  public Command runMotor() {
    return this.run(() -> intakeMotor.set(IntakeConstants.motorSpeed));
  }

  public Command reverseMotor() {
    return this.run(() -> intakeMotor.set(-IntakeConstants.motorSpeed));
  }

  public Command stopMotorCmd() {
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command setVoltage() {
    return this.run(() -> intakeMotor.setVoltage(IntakeConstants.voltage));
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
