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
import monologue.Annotations.Log;
import monologue.Logged;

public class Outtake extends SubsystemBase implements Logged{

  private final SparkMax outtakeMotor;
  private final SparkMaxConfig motorConfig;
  private final DigitalInput frontReceiver;
 private final DigitalInput middleReceiver;
 private final DigitalInput beambreak;
 



  /** Creates a new Outtake. */
  public Outtake() {

    outtakeMotor = new SparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    // motor configs
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);
    motorConfig.idleMode(IdleMode.kBrake); // prevent coral from slipping out of outtake
    outtakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // front reciever is the one farthest away from intake
    frontReceiver = new DigitalInput(Ports.OuttakePorts.FRONT_RECEIVER);
    // back reciever is the one located in outtake
    middleReceiver = new DigitalInput(Ports.OuttakePorts.MIDDLE_RECEIVER);

    beambreak = new DigitalInput(Ports.IntakePorts.BEAM_BREAK);



  }

  /* Commands */
  /**
   * 
   * Positive velocity = intake!
   * 
   * @return command that sets the speed of the outtake motor to take in a coral
   */

  public Command setIntakeCoralSpeedCmd() { // sets velocity
    return this.run(() -> outtakeMotor.set(OuttakeConstants.MOTOR_SPEED));
  }

  /**
   * identical to setIntakeCoralSpeedCmd()
   * @return
   */
  public Command runMotorCmd(){
    return this.run(() -> outtakeMotor.set(OuttakeConstants.MOTOR_SPEED));
  }
/**
 * identical to setOuttakeCoralSpeedCmd()
 * @return
 */
  public Command reverseMotorCmd(){
    return this.run(() -> outtakeMotor.set(-OuttakeConstants.MOTOR_SPEED));
  }

  /**
   * Negative velocity = outtake!
   * 
   * @return a command that sets the speed of the outtake motor to release a coral
   */

  public Command setOuttakeCoralSpeedCmd() {
    return this.run(() -> outtakeMotor.set(-OuttakeConstants.MOTOR_SPEED));
  }

  /**
   * reverse of coral outtake, potentially processor score
   * @return
   */
  public Command reverseOuttakeCmd(){
    return this.run(() -> outtakeMotor.set(-OuttakeConstants.BARGE_MOTOR_SPEED));
  }

  public Command removeAlgaeCmd() {
    return this.run(() -> outtakeMotor.set(-OuttakeConstants.REMOVE_ALGAE_SPEED));
  }

  public Command setVoltageCmd() {
    return this.run(() -> outtakeMotor.setVoltage(OuttakeConstants.VOLTAGE));
  }

  public Command stopMotorCmd() {
    return this.runOnce(() -> outtakeMotor.setVoltage(0));
  }


  public boolean isBeamBroken() {
    // System.out.println("intake");
    return !beambreak.get();
  }
  

  /**
   * 
   * @return true if coral is at the right position in outtake
   */

  @Log.NT
  public boolean isBeamBrokenFront() {
    return !frontReceiver.get();
  }

  @Log.NT
  public boolean isBeamBrokenBack() {
    return !middleReceiver.get();
  }

   /*
   * @ngozi, emily, yujing
   * i think this needs editing, need to look at both intake beam break and
   * outtake beam break. IF the intake BB off while outtake BB on, it'll stop.
   */
  public boolean isCoral() {
    return !isBeamBrokenBack() && isBeamBrokenFront();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("OUT BB front", isBeamBrokenFront());
    SmartDashboard.putBoolean("OUT BB middle", isBeamBrokenBack());
    SmartDashboard.putBoolean("IS CORAL", isCoral());
    SmartDashboard.putBoolean("Intake BB", isBeamBroken());
  }   
}
