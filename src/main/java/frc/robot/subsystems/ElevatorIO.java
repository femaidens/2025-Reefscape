// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ElevatorIO extends AutoCloseable  {
  /** Creates a new ElevatorIO. */

  public double getPosition();

  // public double getVoltage();

  public void setVoltage( double voltage );



}
