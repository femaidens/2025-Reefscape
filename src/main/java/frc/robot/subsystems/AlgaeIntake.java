// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.opencv.imgproc.GeneralizedHoughBallard;

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
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Ports;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new Algae_Intake. */
  private static SparkMax intakePivotLeader;
  private static SparkMax intakePivotFollower;

  private static SparkMax intakeRollerLeader;
  private static SparkMax intakeRollerFollower;
  private static SparkMaxConfig globalConfig;

  private static ProfiledPIDController intakePID;

  private static SimpleMotorFeedforward intakeFF;

  private static RelativeEncoder pivotEncoder;

  public AlgaeIntake() {
    //intake pivot instantiations and setup
    intakePivotLeader = new SparkMax(AlgaeIntakePorts.INTAKE_PIVOT_LEADER, MotorType.kBrushless);
    intakePivotFollower = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_PIVOT_FOLLOWER, MotorType.kBrushless);
    globalConfig = new SparkMaxConfig();

    pivotEncoder = intakePivotFollower.getEncoder();

    globalConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .follow(intakePivotLeader, false);
    globalConfig.encoder
      .positionConversionFactor(AlgaeIntakeConstants.POS_CONVERSION_FACTOR)
      .velocityConversionFactor(AlgaeIntakeConstants.VEL_CONVERSION_FACTOR);
    
    
    intakePivotLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivotFollower.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //intake roller instantiations and setup
    intakeRollerLeader = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_ROLLER_LEADER, MotorType.kBrushless);
    intakeRollerFollower = new SparkMax(Ports.AlgaeIntakePorts.INTAKE_ROLLER_FOLLOWER, MotorType.kBrushless);
    
    globalConfig
      .inverted(true);

    intakeRollerLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    globalConfig
      .follow(intakeRollerLeader, false);

    intakeRollerFollower.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //PID and FF
    intakePID = new ProfiledPIDController(
      ClimbConstants.AlgaeIntakeConstants.PIDConstants.kP, 
      ClimbConstants.AlgaeIntakeConstants.PIDConstants.kI, 
      ClimbConstants.AlgaeIntakeConstants.PIDConstants.kD, 
      new Constraints(ClimbConstants.AlgaeIntakeConstants.PIDConstants.MAX_VELOCITY, ClimbConstants.AlgaeIntakeConstants.PIDConstants.MAX_ACCELERATION)
      );

    intakeFF = new SimpleMotorFeedforward(
      ClimbConstants.AlgaeIntakeConstants.FFConstants.kS, 
      ClimbConstants.AlgaeIntakeConstants.FFConstants.kV, 
      ClimbConstants.AlgaeIntakeConstants.FFConstants.kA);
  }
  /**
   * use PID in order to set angle of algae intake to score processor
   */
  public static void setPID(double setpoint){
    intakePivotLeader.setVoltage(
      intakePID.calculate(pivotEncoder.getPosition()) +  intakeFF.calculate(intakePID.getSetpoint().velocity, setpoint));
    intakePivotFollower.resumeFollowerMode();
  }

/**
 * Set rollers to run
 * @return 
 */
  public Command runRollersCmd(){
    return this.run( () -> intakeRollerLeader.set(AlgaeIntakeConstants.ROLLER_SPEED));
  }

  /**
 * Set rollers to run
 * @return 
 */
public Command reverseRollersCmd(){
  return this.run( () -> intakeRollerLeader.set(-AlgaeIntakeConstants.ROLLER_SPEED));
}

  /**
 * Set 
 * @return
 */
public Command setVoltageCmd(){
  return this.run( () -> intakeRollerLeader.setVoltage(ClimbConstants.AlgaeIntakeConstants.PIVOT_VOLTAGE));
}

  /**
   * 
   * @return command to stop rollers
   */
  public Command stopRollersCmd() {
    return this.run(() -> intakeRollerLeader.setVoltage(0));
  }

  public Command setProcessorCmd() {
    return this.run(() -> this.setPID(ClimbConstants.AlgaeIntakeConstants.PIVOT_SETPOINT));
  }

  public Command intakeAlgaeCmd() {
    return this.run(() -> this.setPID(0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
