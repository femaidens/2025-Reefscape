// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.*;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Ports.*;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax intakeMotor;
  private final PIDController pidController;

  public Intake() {
    intakeMotor = new SparkMax(IntakePorts.INTAKE_MOTOR, MotorType.kBrushless);
    pidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
  }

  public Command runMotor(){
    return this.run(() -> intakeMotor.set(IntakeConstants.motorSpeed));
  } 

  public Command reverseMotor(){
    return this.run(() -> intakeMotor.set(-IntakeConstants.motorSpeed));
  }

  public Command stopMotorCmd(){
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command setVoltageCmd(){
    return this.run(() -> intakeMotor.setVoltage(IntakeConstants.voltage));
  }


  public void intakeCoral(){
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
