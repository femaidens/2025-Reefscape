// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Vector;
import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Ports;
import frc.robot.Ports.*;

public class Drive extends SubsystemBase {

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;

  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // private final AHRS gyro;

  private final PIDController leftPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightPIDController = new PIDController(PID.kP, PID.kI, PID.kD);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
            
  /** Creates a new Drive. */
  public Drive() {
    leftLeader = new SparkMax(Drivetrain.LEFT_LEADER, MotorType.kBrushless);
    leftFollower = new SparkMax(Drivetrain.LEFT_FOLLOWER, MotorType.kBrushless);
    rightLeader = new SparkMax(Drivetrain.RIGHT_LEADER, MotorType.kBrushless);
    rightFollower = new SparkMax(Drivetrain.RIGHT_FOLLOWER, MotorType.kBrushless);

    drive = new DifferentialDrive(leftLeader::set, rightLeader::set);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();

    odometry = new DifferentialDriveOdometry(
      new Rotation2d(), 
      0, 
      0, 
      new Pose2d());


    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // gyro = new AHRS(NavXComType.kMXP_UART); 

    globalConfig
    .idleMode(IdleMode.kBrake);
    
      leftLeaderConfig
      .apply(globalConfig)
      .inverted(true);

      leftFollowerConfig
      .apply(globalConfig)
      .follow(leftLeader);

      leftLeaderConfig.encoder
      .positionConversionFactor(DriveConstants.POSITION_FACTOR)
      .velocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

      leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void drive(double leftSpeed, double rightSpeed) {
    final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
    final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
    
    final double leftFeedforward = feedforward.calculate(realLeftSpeed);
    final double rightFeedforward = feedforward.calculate(realRightSpeed);
  
    final double leftPID = leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
    final double rightPID = rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);

    double leftVoltage = leftPID + leftFeedforward;
    double rightVoltage = rightPID +rightFeedforward;

    leftLeader.setVoltage(leftVoltage);
    rightLeader.setVoltage(rightVoltage);
  }

  private void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  public Command driveCmd(double vLeft, double vRight) {
    return run(() -> drive(vLeft, vRight));
  }

  @Override 
  public void periodic() {
    // updateOdometry(gyro.getRotation2d());
  }

  
}
