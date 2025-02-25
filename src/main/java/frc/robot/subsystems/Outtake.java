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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Ports;

public class Outtake extends SubsystemBase {

  private final SparkMax outtakeMotor;
  private final SparkMaxConfig motorConfig;
  //private final DigitalInput frontReceiver;
  private final DigitalInput receiver;
  private final DigitalInput beamBreak; 



  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    // motor configs
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);
    motorConfig.idleMode(IdleMode.kBrake); // prevent coral from slipping out of outtake
    motorConfig.inverted(true);
    outtakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // front reciever is the one farthest away from intake
    //frontReceiver = new DigitalInput(Ports.BeamBreakPorts.FRONT_RECEIVER);
    // back reciever is the one located in outtake
    receiver = new DigitalInput(Ports.OuttakePorts.RECEIVER);
    beamBreak = new DigitalInput(Ports.IntakePorts.BEAM_BREAK); 



  }

  /* Commands */
  /**
   * 
   * Positive velocity = intake!
   * 
   * @return command that sets the speed of the outtake motor to take in a coral
   */
   // we don't need this anymore
  // public Command setIntakeCoralSpeedCmd() { // sets velocity
  //   return this.run(() -> outtakeMotor.set(OuttakeConstants.OUTTAKE_SPEED));
  // }

  /**
   * Negative velocity = outtake!
   * 
   * @return a command that sets the speed of the outtake motor to release a coral
   */

  public Command setOuttakeCoralSpeedCmd() {
    return this.run(() -> outtakeMotor.set(OuttakeConstants.OUTTAKE_SPEED));
  }

  public Command removeAlgaeCmd() {
    return this.run(() -> outtakeMotor.set(OuttakeConstants.REMOVE_ALGAE_SPEED));
  }

  public Command setVoltageCmd() {
    return this.run(() -> outtakeMotor.setVoltage(OuttakeConstants.VOLTAGE));
  }

  public Command stopMotorCmd() {
    return this.run(() -> outtakeMotor.set(0));
  }

  /**
   * 
   * @return true if coral is at the right position in outtake
   */

  public boolean isBeamBroken() {
    return !receiver.get();
  }

  public boolean isBeamBrokenIntake() {
    return !beamBreak.get();
  }
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Outtake Reciever", isBeamBroken()); 
    SmartDashboard.putBoolean("Intake Beambreak", isBeamBrokenIntake());

  }
}