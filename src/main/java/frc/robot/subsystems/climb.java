package frc.robot.subsystems;
import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;


import com.revrobotics.AbsoluteEncoder;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class climb extends SubsystemBase {
    private final TalonFX LEADER = new TalonFX(Ports.LEADER_PORT);
    private final TalonFX FOLLOWER = new TalonFX (Ports.FOLLOWER_PORT);

    AbsoluteEncoder Encoder;

    public climb(){
        Encoder = LEADER.getAbsoluteEncoder();
    }

    double MAXPosition = 360.0;
    double MINPosition = 0; //subject to change, need to change to rotation??



  public Command climbUPCommand () {
        return this.run (() -> climbUP());
  }

  public Command climbDOWNCommand (){
        return this.run(() -> climbDOWN());
  }

  public void climbUP(){
    {
        if (Encoder.getPosition () < MAXPosition) {
            LEADER.set (ClimbConstants.ClimbSpeed);
            FOLLOWER.set (ClimbConstants.ClimbSpeed);
        } else {
            LEADER.set (0);
            FOLLOWER.set (0);
        }

    }
  }

  public void climbDOWN (){
    {
        if (Encoder.getPosition () > MINPosition){
            LEADER.set(-ClimbConstants.ClimbSpeed);
            FOLLOWER.set(-ClimbConstants.ClimbSpeed);
        } else {
            LEADER.set(0);
            FOLLOWER.set(0);
        }
    }
  }

  public void stopMotors(){
    LEADER.set(0);
    FOLLOWER.set(0);
  }


  
   



}
