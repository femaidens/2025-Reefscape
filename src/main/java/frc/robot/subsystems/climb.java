package frc.robot.subsystems;

//import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
//import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Ports;

public class Climb extends SubsystemBase {
  private final SparkMax follower;
  // private final SparkAbsoluteEncoder followerEncoder;
  private final SparkMaxConfig followerConfig;

  public Climb() {
    follower = new SparkMax(Ports.FOLLOWER_PORT, MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.inverted(true);
    follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command climbFwdCmd() {
    return this.run(() -> follower.set(ClimbConstants.CLIMB_SPEED));
  }

  public Command climbBkwdCmd() {
    return this.run(() -> follower.set(-ClimbConstants.CLIMB_SPEED));
  }

  public Command pulleySystemCmd() {
    return this.run(() -> follower.set(ClimbConstants.CLIMB_SPEED));

  }

  public void stopMotors() {
    follower.set(0);
    //follower.set(0);
  }
}//
