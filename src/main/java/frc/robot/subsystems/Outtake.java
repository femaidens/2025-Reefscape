// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Outtake extends SubsystemBase {

  private final SparkMax outtakeMotor;

  private final RelativeEncoder outtakeEncoder;
  private final Ultrasonic ultrasonic;
  private final PIDController ultrasonicPID;

  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    outtakeEncoder = outtakeMotor.getEncoder();
    ultrasonic = new Ultrasonic(Ports.UltrasonicPorts.UltrasonicPingPort, Ports.UltrasonicPorts.UltrasonicEchoPort);
    ultrasonicPID = new PIDController(
      Constants.UltrasonicConstants.PIDConstants.kP,
      Constants.UltrasonicConstants.PIDConstants.kI,
      Constants.UltrasonicConstants.PIDConstants.kD
      );

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

  public void stopMotor() {
    outtakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
