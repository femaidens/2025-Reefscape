package frc.robot.subsystems;

import frc.robot.Ports;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final SparkMax leader;
  private final SparkMax follower;
  private final SparkAbsoluteEncoder leaderEncoder;
  private final SparkAbsoluteEncoder followerEncoder;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;

  public Climb(){
    leader = new SparkMax(Ports.LEADER_PORT, MotorType.kBrushless);
    follower = new SparkMax(Ports.FOLLOWER_PORT, MotorType.kBrushless);
    leaderEncoder = leader.getAbsoluteEncoder();
    followerEncoder = follower.getAbsoluteEncoder();
    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();
  }
  
  public Command climbFwdCmd () {
    return this.run (() -> {
      double currentRotation = leaderEncoder.getPosition();
        if (currentRotation < Constants.ClimbConstants.MAXRotation) {
            leaderConfig.idleMode(IdleMode.kBrake);
            leader.configure(leaderConfig, null, null);
            leader.set(ClimbConstants.ClimbSpeed);
            followerConfig.idleMode(IdleMode.kCoast);
            follower.configure(followerConfig, null, null);
      } 
        else {
            leader.set (0);
            follower.set (0);
      }
        });
  }

  public Command climbBkwdCmd (){
    return this.run(() -> {
      double currentRotation = leaderEncoder.getPosition();
        if (currentRotation > Constants.ClimbConstants.MINRotation) {
            leaderConfig.idleMode(IdleMode.kBrake);
            leader.configure(leaderConfig, null, null);
            leader.set(ClimbConstants.ClimbSpeed);
            followerConfig.idleMode(IdleMode.kCoast);
            follower.configure(followerConfig, null, null);
      }  
        else {
            leader.set (0);
            follower.set (0);
      }
        });
  }

  public Command pulleySystemCmd (){
    return this.run(() -> {
      double currentRotation = followerEncoder.getPosition();
        if(currentRotation < Constants.ClimbConstants.MAXRotation) {
          leaderConfig.idleMode(IdleMode.kBrake);
          leader.configure(leaderConfig, null, null);
          followerConfig.idleMode(IdleMode.kBrake);
          follower.configure(followerConfig, null, null);
          follower.set(ClimbConstants.ClimbSpeed);
      }
        else {
            leader.set(0);
            follower.set(0);
        }
    });
    

  }

  public void stopMotors(){
    leader.set(0);
    follower.set(0);
  }
}
