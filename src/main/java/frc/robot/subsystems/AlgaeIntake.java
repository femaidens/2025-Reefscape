// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// import org.opencv.imgproc.GeneralizedHoughBallard;

import com.revrobotics.RelativeEncoder;
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

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new Algae_Intake. */
  // private static SparkMax intakePivotLeader;
  // private static SparkMax intakePivotFollower;

  private static SparkMax intakeRollerLeader;
  private static SparkMax intakeRollerFollower;
  private static SparkMaxConfig globalConfig;

  private static ProfiledPIDController intakePID;

  private static SimpleMotorFeedforward intakeFF;

  private static RelativeEncoder pivotEncoder;

  public AlgaeIntake() {
    //intake pivot instantiations and setup
    // intakePivotLeader = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_PIVOT_LEADER, MotorType.kBrushless);
    // intakePivotFollower = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_PIVOT_FOLLOWER, MotorType.kBrushless);
    globalConfig = new SparkMaxConfig();

    // pivotEncoder = intakePivotFollower.getEncoder();

    // globalConfig
    //   .inverted(true)
    //   .idleMode(IdleMode.kBrake)
    //   .follow(intakePivotLeader, false);
    globalConfig.encoder
      .positionConversionFactor(Constants.AlgaeIntakeConstants.POSITION_CONVERSIONFACTOR)
      .velocityConversionFactor(Constants.AlgaeIntakeConstants.VELOCITY_CONVERSIONFACTOR);
    
    
    // intakePivotLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // intakePivotFollower.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //intake roller instantiations and setup
    intakeRollerLeader = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_ROLLER_LEADER, MotorType.kBrushless);
    intakeRollerFollower = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_ROLLER_FOLLOWER, MotorType.kBrushless);
    
    intakeRollerLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    globalConfig
      .follow(intakeRollerLeader, false);
      
    intakeRollerFollower.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //PID and FF
    intakePID = new ProfiledPIDController(
      Constants.AlgaeIntakeConstants.PIDConstants.kP, 
      Constants.AlgaeIntakeConstants.PIDConstants.kI, 
      Constants.AlgaeIntakeConstants.PIDConstants.kD, 
      new Constraints(Constants.AlgaeIntakeConstants.PIDConstants.MAX_VELOCITY, Constants.AlgaeIntakeConstants.PIDConstants.MAX_ACCELERATION)
      );

    intakeFF = new SimpleMotorFeedforward(
      Constants.AlgaeIntakeConstants.FFConstants.kS, 
      Constants.AlgaeIntakeConstants.FFConstants.kV, 
      Constants.AlgaeIntakeConstants.FFConstants.kA);
  }
  /**
   * use PID in order to set angle of algae intake to score processor
   */
  // public static void setPID(double setpoint){
  //   intakePivotLeader.setVoltage(
  //     intakePID.calculate(pivotEncoder.getPosition()) +  intakeFF.calculate(intakePID.getSetpoint().velocity, setpoint));
  //   intakePivotFollower.resumeFollowerMode();
  // }

/**
 * 
 * @return command to run rollers
 */
  public Command runRollers(){
    return this.run(() -> intakeRollerLeader.set(0.3));
  }

  /**
   * 
   * @return command to stop rollers
   */
  public Command stopRollers() {
    return this.run(() -> intakeRollerLeader.set(0));
  }

  public Command reverseRollers() {
    return this.run(() -> intakeRollerLeader.set(-0.3));
  }

  // public Command setProcessor() {
  //   return this.run(() -> this.setPID(Constants.AlgaeIntakeConstants.PIVOT_SETPOINT));
  // }

  // public Command setGround(){
  //   return this.run(() -> this.setPID(Constants.AlgaeIntakeConstants.PIVOT_GROUND_SETPOINT));
  // }

  // public Command intakeAlgae() {
  //   return this.run(() -> this.setPID(0));
  // }

  
  
  public void initDefaultCommand() {
    // this.setProcessor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}