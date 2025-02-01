// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Algae_Intake extends SubsystemBase {
  /** Creates a new Algae_Intake. */
  private static SparkMax intakePivotLeader;
  private static SparkMax intakePivotFollower;

  private static SparkMax intakeRollerLeader;
  private static SparkMax intakeRollerFollower;

  private static SparkMaxConfig pivotConfig;
  private static SparkMaxConfig rollerConfig;
  private static ProfiledPIDController intakePID;

  private static SimpleMotorFeedforward intakeFF;

  public Algae_Intake() {
    
    intakePivotLeader = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_PIVOT_LEADER, MotorType.kBrushless);
    intakePivotFollower = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_PIVOT_FOLLOWER, MotorType.kBrushless);

    intakeRollerLeader = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_ROLLER_LEADER, MotorType.kBrushless);
    intakeRollerFollower = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_ROLLER_FOLLOWER, MotorType.kBrushless);

    pivotConfig = new SparkMaxConfig();
    rollerConfig = new SparkMaxConfig();

    pivotConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    pivotConfig.encoder
      .positionConversionFactor(Constants.AlgaeIntakeConstants.POSITIONCONVERSIONFACTOR)
      .velocityConversionFactor(Constants.AlgaeIntakeConstants.VELOCITYCONVERSIONFACTOR);
    pivotConfig
      .follow(intakePivotLeader, false);
    
    intakePivotLeader.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivotFollower.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    // rollerConfig.encoder
    //   .positionConversionFactor(Constants.AlgaeIntakeConstants.POSITIONCONVERSIONFACTOR)
    //   .velocityConversionFactor(Constants.AlgaeIntakeConstants.VELOCITYCONVERSIONFACTOR);
    rollerConfig
      .follow(intakeRollerLeader, false);
    
    intakeRollerLeader.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRollerFollower.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakePID = new ProfiledPIDController(
      Constants.AlgaeIntakeConstants.PIDConstants.kP, 
      Constants.AlgaeIntakeConstants.PIDConstants.kI, 
      Constants.AlgaeIntakeConstants.PIDConstants.kD, 
      new Constraints(Constants.AlgaeIntakeConstants.PIDConstants.maxVelocity, Constants.AlgaeIntakeConstants.PIDConstants.maxAcceleration)
      );

    intakeFF = new SimpleMotorFeedforward(
      Constants.AlgaeIntakeConstants.FFConstants.kS, 
      Constants.AlgaeIntakeConstants.FFConstants.kV, 
      Constants.AlgaeIntakeConstants.FFConstants.kA);
  }

  public Command runRollers(){
    return this.run( () -> intakeRollerLeader.setVoltage(Constants.AlgaeIntakeConstants.pivotVoltage));
  }

  public Command stopRollers() {
    return this.run(() -> intakeRollerLeader.setVoltage(0));
  }

  

  


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
