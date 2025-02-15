package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Ports;

public class Climb extends SubsystemBase {
  private final SparkMax climbMotor;
  private final SparkAbsoluteEncoder motorEncoder;
  private final SparkMaxConfig motorConfig;
  private final AbsoluteEncoderConfig encoderConfig;

  public Climb() {
    climbMotor = new SparkMax(Ports.ClimbPorts.CLIMB_MOTOR_PORT, MotorType.kBrushless);
    motorEncoder = climbMotor.getAbsoluteEncoder();
    motorConfig = new SparkMaxConfig();
    encoderConfig = new AbsoluteEncoderConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);
    encoderConfig.positionConversionFactor(360);
    motorConfig.apply(encoderConfig);
    climbMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // public Command climbFwdCmd() {
  //   return this.run(() -> {
  //     double currentRotation = leaderEncoder.getPosition();
  //     if (currentRotation < Constants.ClimbConstants.MAX_ROTATION) {
  //       leader.set(ClimbConstants.CLIMB_SPEED);

  //     } else {
  //       leader.set(0);
  //       follower.set(0);
  //     }
  //   });
  // }

  // public Command climbBkwdCmd() {
  //   return this.run(() -> {
  //     double currentRotation = leaderEncoder.getPosition();
  //     if (currentRotation > Constants.ClimbConstants.MIN_ROTATION) {

  //       leader.set(ClimbConstants.CLIMB_SPEED);
        
  //     } else {
  //       stopMotors();
  //     }
  //   });
  // }

  public Command pulleySystemCmd() {
    return this.run(() -> {
      double currentRotation = motorEncoder.getPosition();
      if (currentRotation < Constants.ClimbConstants.MAX_ROTATION) {
        climbMotor.set(ClimbConstants.CLIMB_SPEED);
      } else {
        stopMotorsCmd();
      }
    });

  }

  public Command stopMotorsCmd() {
    return this.run(() -> climbMotor.set(0));
    //follower.set(0);
  }
}
