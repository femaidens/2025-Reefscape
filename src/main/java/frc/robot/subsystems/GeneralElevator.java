// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ActualElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NoElevator;

public class GeneralElevator extends SubsystemBase implements ElevatorIO{
  /** Creates a new GeneralElevator. */
  private ElevatorIO elevator = (Robot.isReal()) ? (new ActualElevator()) : (new Elevator());
  // private ElevatorIO elevator = new NoElevator();
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() throws Exception {
    elevator.close();
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  // @Override
  // public double getVelocityy() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getVelocityy'");
  // }

  @Override
  public void setVoltage(double setpoint) {
    elevator.setVoltage(setpoint);
    // elevator.setVoltage(elevatorPID.calculate( elevatorEncoder.getPosition() ) + 
    //     ff.calculate( elevatorPID.calculate(elevatorEncoder.getPosition()) ) ); // not sure if this is correct
        // elevatorMotorFollower.resumeFollowerMode();
  }
}
