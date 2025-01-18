// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // this might 
  // public void outtakeCoral() {
  //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.coralSetpoint);
  //     if(ultrasonicPID.atSetpoint()) {
  //       runMotor();
  //     }
  //     stopMotor();
  //   }

    public boolean isCoral() {
      if (ultrasonicPID.atSetpoint()) {
        return true;
      }
      return false;
    }

  public void intakeAlgae() {
    ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.algaeSetpoint);
      if(ultrasonicPID.atSetpoint()) {
        stopMotor();
      }
      reverseMotor();
  }

  public void runMotor() {
    outtakeMotor.set(Constants.OutakeConstants.motorSpeed);
  }

  public void reverseMotor() {
    outtakeMotor.set(-Constants.OutakeConstants.motorSpeed);
  }

  public void setVoltage() {
    outtakeMotor.setVoltage(0);
  }

  public double getDistance() {
    double measurement = ultrasonic.getRangeInches();
    double filteredMeasurement = filter.calculate(measurement);
    return ultrasonicPID.calculate(filteredMeasurement);
  }

  public void stopMotor() {
    outtakeMotor.setVoltage(0);
  }

  /*  testing more command based code */
  
  // public Command outakeCoral() {
  //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.coralSetpoint);
  //   if(ultrasonicPID.atSetpoint()) {
  //     return this.run(()-> runMotor());
  //   }
  //   return this.run(()->stopMotor());
  // }

  // public Command intakeAlgaes() {
  //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.algaeSetpoint);
  //     if(ultrasonicPID.atSetpoint()) {
  //       return this.run(()-> reverseMotors());
  //     }
  //     return this.run(()->stopMotor());
   
  // }


  // public Command runMotors() {
  //   return this.run(()-> outtakeMotor.set(Constants.OutakeConstants.motorSpeed));
  // }

  // public Command reverseMotors() {
  //   return this.run(()-> outtakeMotor.set(-Constants.OutakeConstants.motorSpeed));
  // }

  // public Command setVoltages() {
  //   return this.run(()-> outtakeMotor.setVoltage(Constants.OutakeConstants.voltage));
  // }

  // public Command stopMotors() {
  //   return this.run(()-> outtakeMotor.setVoltage(Constants.OutakeConstants.voltage));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", getDistance());
  }
}
