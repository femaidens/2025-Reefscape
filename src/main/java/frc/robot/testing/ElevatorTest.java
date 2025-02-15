// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.GeneralElevator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.ParameterizedTest;
// import org.junit.jupiter.params.provider.Arguments;
// import org.junit.jupiter.params.provider.MethodSource;
// import frc.robot.testing.UnitTestingUtil.*;

import org.junit.jupiter.api.Test;
public class ElevatorTest{
  private GeneralElevator elevator;

  @BeforeEach
  private void setUpTests() {
    assert HAL.initialize(500,0);
    elevator = GeneralElevator.create();
  }

  @AfterEach
  public void destroy() throws Exception{
     CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().cancelAll();
    elevator.close();
  }

  @Test
  public void test(){
    UnitTestingUtil.run(elevator.reachGoal(Constants.ElevatorConstants.SetpointConstants.FIRST_LVL));
    UnitTestingUtil.fastForward(500);
    assertEquals(4, elevator.position());
  }

  
  

}