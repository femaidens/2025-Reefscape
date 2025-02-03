// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Ports;

public class Outtake extends SubsystemBase {

  private final SparkMax outtakeMotor;
  private final SparkMaxConfig motorConfig;
  private final DigitalInput frontReceiver;
  private final DigitalInput backReceiver;

  // private final Ultrasonic ultrasonic;
  // private final PIDController ultrasonicPID;
  // private final MedianFilter filter;

  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    //motor configs
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);
    motorConfig.idleMode(IdleMode.kBrake); //prevent coral from slipping out of outtake
    outtakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  
    frontReceiver = new DigitalInput(Ports.BeamBreakPorts.FRONT_RECEIVER);
    backReceiver = new DigitalInput(Ports.BeamBreakPorts.BACK_RECEIVER);
    
   
    

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

   public Command setIntakeCoralSpeed() { //sets velocity
     return this.run(()-> outtakeMotor.set(OuttakeConstants.MOTOR_SPEED));
    }

  /**
   * Negative velocity = outtake!
   * 
   * @return a command that sets the speed of the outtake motor to release a coral
   */

  public Command setOuttakeCoralSpeed() {
  return this.run(()-> outtakeMotor.set(-OuttakeConstants.MOTOR_SPEED));
  }

  public Command removeAlgae() {
    return this.run(()-> outtakeMotor.set(OuttakeConstants.REMOVE_ALGAE_SPEED));
  }

  public Command setVoltage() {
    return this.run(()-> outtakeMotor.setVoltage(OuttakeConstants.VOLTAGE));
  }

  public Command stopMotor() {
    return this.run(()-> outtakeMotor.setVoltage(0));
  }

  /**
   * 
   * @return true if coral is at the right position in outtake 
   */

  public boolean isCoral() {
  if (frontReceiver.get() == false && backReceiver.get()) {
      return true;
    }
      return false;
  }

   // not using ultrasonic anymore ;( 
  // public double getDistance() {
  //   double measurement = ultrasonic.getRangeInches();
  //   double filteredMeasurement = filter.calculate(measurement);
  //   return ultrasonicPID.calculate(filteredMeasurement);
  // }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Distance", getDistance());
  }
}
