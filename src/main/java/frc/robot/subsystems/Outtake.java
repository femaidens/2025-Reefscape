// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Outtake extends SubsystemBase {

  private final SparkMax outtakeMotor;
  private final DigitalInput frontReciever;
  private final DigitalInput backReciever;

  // private final Ultrasonic ultrasonic;
  // private final PIDController ultrasonicPID;
  // private final MedianFilter filter;

  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    frontReciever = new DigitalInput(Ports.BeamBreakPorts.FRONT_RECIEVER);
    backReciever = new DigitalInput(Ports.BeamBreakPorts.BACK_RECIEVER);

    // ultrasonic = new Ultrasonic(Ports.UltrasonicPorts.UltrasonicPingPort, Ports.UltrasonicPorts.UltrasonicEchoPort);
    // ultrasonicPID = new PIDController(
    //   Constants.UltrasonicConstants.PIDConstants.kP,
    //   Constants.UltrasonicConstants.PIDConstants.kI,
    //   Constants.UltrasonicConstants.PIDConstants.kD
    //   );
    // filter = new MedianFilter(5);
   
  }

  /* Commands */


  /**
   * 
   * Positive velocity = intake! 
   * 
   * @return command that sets the speed of the outtake motor to take in a coral 
   */

  public Command setIntakeCoralSpeedCmd() {
    return this.run(()-> setIntakeCoralSpeed());
  }

  /**
   * Negative velocity = outtake!
   * 
   * @return a command that sets the speed of the outtake motor to release a coral
   */

  public Command setOuttakeCoralSpeedCmd() {
    return this.run(()-> setOuttakeCoralSpeed());
  }

  public Command setVoltageCmd() {
    return this.run(()-> setVoltage());
  }

  public Command stopMotorCmd() {
    return this.runOnce(()-> stopMotor());
  }
  

  public void setIntakeCoralSpeed() { //sets velocity
    outtakeMotor.set(Constants.OuttakeConstants.motorSpeed);
  }

  public void setOuttakeCoralSpeed() {
    outtakeMotor.set(-Constants.OuttakeConstants.motorSpeed); //negative value outtakes the coral
  }

  /**
   * 
   * @return true if coral is at the right position in outtake
   */

  public boolean isCoral() {
  if (frontReciever.get() == false && backReciever.get()) {
      return true;
    }
      return false;
  }

  public void setVoltage() {
    outtakeMotor.setVoltage(Constants.OuttakeConstants.voltage);
  }

  public void stopMotor() {
    outtakeMotor.setVoltage(0);
  }

  /* COMMANDS (NO METHODS) */
  // public Command setIntakeCoralSpeed() {
  //   return this.run(()-> outtakeMotor.set(Constants.OutakeConstants.motorSpeed));
  // }

  // public Command setOuttakeCoralSpeed() {
  //   return this.run(()-> outtakeMotor.set(-Constants.OutakeConstants.motorSpeed));
  // }

  // public Command setVoltage() {
  //   return this.run(()-> outtakeMotor.setVoltage(Constants.OutakeConstants.voltage));
  // }

  // public Command stopMotor() {
  //   return this.run(()-> outtakeMotor.setVoltage(Constants.OutakeConstants.voltage));
  // }

   // not using ultrasonic anymore ;( 
  // public double getDistance() {
  //   double measurement = ultrasonic.getRangeInches();
  //   double filteredMeasurement = filter.calculate(measurement);
  //   return ultrasonicPID.calculate(filteredMeasurement);
  // }

  // this might not work
  // public void outtakeCoral() {
  //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.coralSetpoint);
  //     if(ultrasonicPID.atSetpoint()) {
  //       runMotor();
  //     }
  //     stopMotor();
  //   }

  /**
   * 
   * @return true if coral is detected passing through from intake to outtake
   */

    // public boolean isCoral() {
    //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.coralSetpoint);
    //   if (ultrasonicPID.atSetpoint()) {
    //     return true;
    //   }
    //   return false;
    // }

    // public boolean isAlgae() {
    //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.algaeSetpoint);
    //   if (ultrasonicPID.atSetpoint()) {
    //     return true;
    //   }

    //   return false;
    // }
  // do not need
  // public void intakeAlgae() {
  //   ultrasonicPID.setSetpoint(Constants.UltrasonicConstants.algaeSetpoint);
  //     if(ultrasonicPID.atSetpoint()) {
  //       stopMotor();
  //     }
  //     reverseMotor();
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Distance", getDistance());
  }
}
