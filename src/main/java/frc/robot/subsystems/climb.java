package frc.robot.subsystems;
import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class climb extends SubsystemBase {
    private final TalonFX LEADER = new TalonFX(Ports.LEADER_PORT);
    private final TalonFX FOLLOWER = new TalonFX (Ports.FOLLOWER_PORT);

    double MAXRotation = 90.0;
    double MINRotation = 0; //subject to change, need to change to rotation??

    public double getSelectedMotorRotation(TalonFX motor){
      final double TICKS_PER_REVOLUTION = 1000;
      double ticks = LEADER.getSelectedSensorPosition();
    // use the encoders whatamidoing
      return (ticks / TICKS_PER_REVOLUTION) * 360;
    }


  
    public Command climbUPCommand () {
        return this.run (() -> climbUP());
  }

  public Command climbDOWNCommand (){
        return this.run(() -> climbDOWN());
  }

  public void climbUP(){
    double currentRotation = getSelectedMotorRotation();
        if (currentRotation < MAXRotation) {
            LEADER.set (ClimbConstants.ClimbSpeed);
            FOLLOWER.set (ClimbConstants.ClimbSpeed);
        } else {
            LEADER.set (0);
            FOLLOWER.set (0);
        }

    }

  public void climbDOWN (){
        double currentRotation = getSelectedMotorRotation();
        if (currentRotation > MINRotation){
            LEADER.set(-ClimbConstants.ClimbSpeed);
            FOLLOWER.set(-ClimbConstants.ClimbSpeed);
        } else {
            LEADER.set(0);
            FOLLOWER.set(0);
        }
    }
  

  public void stopMotors(){
    LEADER.set(0);
    FOLLOWER.set(0);
  }
}
