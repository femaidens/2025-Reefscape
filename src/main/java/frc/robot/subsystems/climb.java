package frc.robot.subsystems;

import frc.robot.Ports;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class climb extends SubsystemBase {
  private final TalonFX leader;
  private final TalonFX follower;
  private final Encoder encoder;

  public climb(){
    leader = new TalonFX(Ports.LEADER_PORT);
    follower = new TalonFX(Ports.FOLLOWER_PORT);
    encoder = new Encoder(Ports.channelA, Ports.channelB);
    leader.setNeutralMode(NeutralModeValue.Brake);
  }

  public int get(){
    final double TICKS_PER_REVOLUTION = 1000;
    int ticks = encoder.get(); 
    return (int) (ticks / TICKS_PER_REVOLUTION) * 360;
  }
  
    public Command climbFwdCmd () {
        return this.run (() -> {
          double currentRotation = get();
          if (currentRotation < Constants.ClimbConstants.MAXRotation) {
            leader.set (ClimbConstants.ClimbSpeed);
        return this.run (() -> climbFORWARD());
  }

    public Command setBrakeModeCmd(){
      return this.run(() -> setBrakeMode());
    }

    public Command setCoastModeCmd() {
      return this.run(()-> setCoastMode());
    }

  public Command climbBkwdCmd (){
        return this.run(() -> climbBACKWARD());
  }

  public void climbFORWARD(){
    double currentRotation = get();
        if (currentRotation < Constants.ClimbConstants.MAXRotation) {
            leader.setNeutralMode(NeutralModeValue.Brake);
            follower.setNeutralMode(NeutralModeValue.Coast);
         } 
         else {
            leader.set (0);
            follower.set (0);
        }
        };
  

  public Command climbBkwdCmd (){
        return this.run(() -> {
          double currentRotation = get();
          if (currentRotation < Constants.ClimbConstants.MAXRotation) {
            leader.set (ClimbConstants.ClimbSpeed);
            follower.setNeutralMode(NeutralModeValue.Coast);
          }  
          else {
            leader.set (0);
            follower.set (0);
          }
        });
  }



  public void stopMotors(){
    leader.set(0);
    follower.set(0);
  }
}
