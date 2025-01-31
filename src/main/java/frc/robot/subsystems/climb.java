package frc.robot.subsystems;

import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climb extends SubsystemBase {
  private final TalonFX leader;
  private final TalonFX follower;
  private final Encoder encoder;

  public climb(){
    leader = new TalonFX(Ports.LEADER_PORT);
    follower = new TalonFX(Ports.FOLLOWER_PORT);
    encoder = new Encoder(Ports.channelA, Ports.channelB);
  }

  public int get(){
    final double TICKS_PER_REVOLUTION = 1000;
    int ticks = encoder.get(); 
    return (int) (ticks / TICKS_PER_REVOLUTION) * 360;
  }
  
    public Command climbUPCommand () {
        return this.run (() -> climbUP());
  }

  public Command climbDOWNCommand (){
        return this.run(() -> climbDOWN());
  }

  public void climbUP(){
    double currentRotation = get();
        if (currentRotation < Constants.ClimbConstants.MAXRotation) {
            leader.set (ClimbConstants.ClimbSpeed);
            follower.set (ClimbConstants.ClimbSpeed);
        } else {
            leader.set (0);
            follower.set (0);
        }

    }

  public void climbDOWN (){
        double currentRotation = get();
        if (currentRotation > Constants.ClimbConstants.MINRotation){
            leader.set(-ClimbConstants.ClimbSpeed);
            follower.set(-ClimbConstants.ClimbSpeed);
        } else {
            leader.set(0);
            follower.set(0);
        }
    }
  

  public void stopMotors(){
    leader.set(0);
    follower.set(0);
  }
}
