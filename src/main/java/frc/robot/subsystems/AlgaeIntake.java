// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import frc.robot.Ports.*;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new Algae_Intake. */

  private static SparkMax intakeRollerLeader;
  private static SparkMax intakeRollerFollower;
  private static SparkMaxConfig globalConfig;

  public AlgaeIntake() {
    //intake pivot instantiations and setup
    
    globalConfig = new SparkMaxConfig();

    //intake roller instantiations and setup
    intakeRollerLeader = new SparkMax(AlgaeIntakePorts.INTAKE_ROLLER_LEADER, MotorType.kBrushless);
    intakeRollerFollower = new SparkMax(AlgaeIntakePorts.INTAKE_ROLLER_FOLLOWER, MotorType.kBrushless);
    
    globalConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);


    intakeRollerLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    globalConfig
      .follow(intakeRollerLeader, false);

    intakeRollerFollower.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }
  

  /**
   * Set rollers to run algae in
   * @return 
   */
    public Command runRollersCmd(){
      return this.run( () -> intakeRollerLeader.set(AlgaeIntakeConstants.ROLLER_SPEED));
    }

  /**
   * Set rollers to run algae out
   * @return 
   */
  public Command reverseRollersCmd(){
    return this.run( () -> intakeRollerLeader.set(-AlgaeIntakeConstants.ROLLER_SPEED));
  }

  /**
   * Set voltage of rollers
   * @return
   */
  public Command setVoltageCmd(){
    return this.run( () -> intakeRollerLeader.setVoltage(AlgaeIntakeConstants.ROLLER_VOLTAGE));
  }

  /**
   * 
   * @return command to stop rollers
   */
  public Command stopRollersCmd() {
    return this.run(() -> intakeRollerLeader.setVoltage(0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
