// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoElevator extends SubsystemBase implements ElevatorIO{
  /** Creates a new NoElevator. */
  public NoElevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() throws Exception {
    //theres nothing.
  }

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocityy() {
    return 0;
  }

  @Override
  public void setVoltage(double voltage) {
    //do nothing
  }
}
