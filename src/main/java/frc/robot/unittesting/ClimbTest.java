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
// import com.revrobotics.spark.SparkAbsoluteEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Ports;

public class climbTest extends SubsystemBase implements AutoCloseable {
  private final SparkMax leader;
  private final SparkMax follower;
  private final SparkAbsoluteEncoder leaderEncoder;
  // private final SparkAbsoluteEncoder followerEncoder;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;
  private final AbsoluteEncoderConfig encoderConfig;

  public climbTest() {
    leader = new SparkMax(Ports.ClimbPorts.LEADER_PORT, MotorType.kBrushless);
    follower = new SparkMax(Ports.ClimbPorts.FOLLOWER_PORT, MotorType.kBrushless);

    leaderEncoder = leader.getAbsoluteEncoder();
    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();
    encoderConfig = new AbsoluteEncoderConfig();
    leaderConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake); 
    followerConfig.follow(leader);
    followerConfig.inverted(true);
    leaderConfig.inverted(false);
    encoderConfig.positionConversionFactor(360);
    leaderConfig.apply(encoderConfig);
    leader.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void close() {
    leader.close();
    follower.close();
  }

  public Command climbFwdCmd() {
    return this.run(() -> {
      double currentRotation = leaderEncoder.getPosition();
      if (currentRotation < Constants.ClimbConstants.MAX_ROTATION) {
        leader.set(ClimbConstants.CLIMB_SPEED);

      } else {
        leader.set(0);
        follower.set(0);
      }
    });
  }

  public Command climbBkwdCmd() {
    return this.run(() -> {
      double currentRotation = leaderEncoder.getPosition();
      if (currentRotation > Constants.ClimbConstants.MIN_ROTATION) {

        leader.set(ClimbConstants.CLIMB_SPEED);
        
      } else {
        stopMotors();
      }
    });
  }

  public Command pulleySystemCmd() {
    return this.run(() -> {
      double currentRotation = leaderEncoder.getPosition();
      if (currentRotation < Constants.ClimbConstants.MAX_ROTATION) {
        follower.set(ClimbConstants.CLIMB_SPEED);
      } else {
        stopMotors();
      }
    });

  }

  public void stopMotors() {
    leader.set(0);
    //follower.set(0);
  }
}
