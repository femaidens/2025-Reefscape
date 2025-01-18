// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Outtake extends SubsystemBase {

  private final SparkMax outtakeMotor;

  private final Ultrasonic ultrasonic;
  private final PIDController ultrasonicPID;
  private final MedianFilter filter;

  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    ultrasonic = new Ultrasonic(Ports.UltrasonicPorts.UltrasonicPingPort, Ports.UltrasonicPorts.UltrasonicEchoPort);
    ultrasonicPID = new PIDController(
      Constants.UltrasonicConstants.PIDConstants.kP,
      Constants.UltrasonicConstants.PIDConstants.kI,
      Constants.UltrasonicConstants.PIDConstants.kD
      );
    filter = new MedianFilter(5);
    ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.setpoint);


  }

  /* Commands */

  public Command runMotorCmd() {
    return this.run(()-> runMotor());
  }

  public Command reverseMotorCmd() {
    return this.run(()-> reverseMotor());
  }

  public Command setVoltageCmd() {
    return this.run(()-> setVoltage());
  }

  public Command stopMotorCmd() {
    return this.run(()-> stopMotor());
  }

  public void outtakeCoral() {
      if(ultrasonicPID.atSetpoint()) {
        reverseMotor();
      }
      stopMotor();
    }

  public void runMotor() {
    outtakeMotor.set(Constants.OutakeConstants.motorSpeed);
  }

  public void reverseMotor() {
    outtakeMotor.set(-Constants.OutakeConstants.motorSpeed);
  }

  public void setVoltage() {
    outtakeMotor.setVoltage(Constants.OutakeConstants.voltage);
  }

  public double getDistance() {
    double measurement = ultrasonic.getRangeInches();
    double filteredMeasurement = filter.calculate(measurement);
    return ultrasonicPID.calculate(filteredMeasurement);
  }

  public void stopMotor() {
    outtakeMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
