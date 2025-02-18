// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Climb;

/** Add your docs here. */
public class Climbing {
  IntakePivot intakePivot; 
  Climb climb; 

  public Climbing(IntakePivot intakePivot, Climb climb){
    this.intakePivot = intakePivot; 
    this.climb = climb; 

  }



  public Command toclimb(){
    return Commands.beforeStarting(intakePivot.pulleySystemCmd()) 
        .andThen(climb.pulleySystemCmd())
        .andThen(climb.stopMotors());
      }
    
}
