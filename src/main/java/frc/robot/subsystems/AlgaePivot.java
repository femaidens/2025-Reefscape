// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Ports.AlgaePivotPorts;

public class AlgaePivot extends SubsystemBase {
  private static SparkMax intakePivotLeader;
  // private static SparkMax intakePivotFollower;

  private static RelativeEncoder pivotEncoder;

  private static SparkMaxConfig globalConfig;

  private static PIDController intakePID;

  // private static SimpleMotorFeedforward intakeFF;

  /** Creates a new AlgaePivot. */
  public AlgaePivot() {
    intakePivotLeader = new SparkMax(AlgaePivotPorts.PIVOT_LEADER, MotorType.kBrushless);

    pivotEncoder = intakePivotLeader.getEncoder();

    globalConfig = new SparkMaxConfig();

    globalConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    globalConfig.encoder
      .positionConversionFactor(AlgaePivotConstants.POS_CONVERSION_FACTOR)
      .velocityConversionFactor(AlgaePivotConstants.VEL_CONVERSION_FACTOR);
    
    intakePivotLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //PID and FF
    intakePID = new PIDController(
      AlgaePivotConstants.PIDConstants.kP, 
      AlgaePivotConstants.PIDConstants.kI, 
      AlgaePivotConstants.PIDConstants.kD
      // new Constraints(AlgaePivotConstants.PIDConstants.MAX_VELOCITY, AlgaePivotConstants.PIDConstants.MAX_ACCELERATION)
      );

    // intakeFF = new SimpleMotorFeedforward(
    //   Constants.AlgaeIntakeConstants.FFConstants.kS, 
    //   Constants.AlgaeIntakeConstants.FFConstants.kV, 
    //   Constants.AlgaeIntakeConstants.FFConstants.kA);
  }

  /**
   * PID in order to set angle of algae intake to score processor
   */
  public void setPID(double setpoint){
    intakePivotLeader.setVoltage(
      intakePID.calculate(pivotEncoder.getPosition(), setpoint)); // +  intakeFF.calculate(intakePID.getSetpoint().velocity, setpoint));
    // intakePivotFollower.resumeFollowerMode();
    System.out.println(intakePID.calculate(pivotEncoder.getPosition(), setpoint));
  }

  public Command setProcessorCmd() {
    return this.run(() -> setPID(AlgaePivotConstants.PROCESSOR_SETPOINT));
  }

  public Command setGroundCmd(){
    return this.run(() -> this.setPID(AlgaePivotConstants.GROUND_SETPOINT));
  }

  public Command intakeAlgaeCmd() {
    return this.run(() -> setPID(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pivotEncoder.getPosition());
    SmartDashboard.putNumber("encoder", pivotEncoder.getPosition());
    
  }
}
