// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.*;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import frc.robot.Ports.*;
// import frc.robot.Constants.*;

// public class IntakePivot extends SubsystemBase {
//   /** Creates a new IntakePivot. */
//   private final SparkMax intakePivotMotor;
//   private final RelativeEncoder pivotEncoder;
//   private final SparkMaxConfig pivotConfig;
//   private final PIDController pivotPID;
//   private final ArmFeedforward pivotFF;

//   public IntakePivot() {
//     intakePivotMotor = new SparkMax(IntakePorts.PIVOT_MOTOR, MotorType.kBrushless);
//     pivotEncoder = intakePivotMotor.getEncoder();
//     pivotConfig = new SparkMaxConfig();
    
//     pivotPID = new PIDController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD);
//     pivotFF = new ArmFeedforward(IntakePivotConstants.FeedForwardConstants.kS, IntakePivotConstants.FeedForwardConstants.kG, IntakePivotConstants.FeedForwardConstants.kV);


//     // Configuring motor
//     pivotConfig
//         .inverted(true)
//         .idleMode(IdleMode.kBrake);
//     pivotConfig.encoder
//         .positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR)
//         .velocityConversionFactor(IntakeConstants.VELOCITY_CONVERSION_FACTOR);
//     pivotConfig.closedLoop
//         .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
//     pivotConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
//     intakePivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//   }

//   public void setPivotPID(double setpoint) {
//     intakePivotMotor.setVoltage(
//         pivotPID.calculate(pivotEncoder.getPosition(), setpoint) + pivotFF.calculate(pivotPID.getSetpoint(), setpoint));
//   }

//   public Command setAngle(double setpoint) {
//     return this.run(() -> setPivotPID(setpoint));
//   }

//   public Command runMotorCmd() {
//     return this.run(() -> intakePivotMotor.set(IntakePivotConstants.PIVOT_SPEED));
//   }

//   public Command reverseMotorCmd() {
//     return this.run(() -> intakePivotMotor.set(-IntakePivotConstants.PIVOT_SPEED));
//   }

//   public Command pulleySystemCmd() {
//     return this.run(() -> intakePivotMotor.set(ClimbConstants.CLIMB_SPEED));
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }

