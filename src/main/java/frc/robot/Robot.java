// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {
  private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  //  private Command m_autonomousCommand;
  // private RobotContainer robotContainer;
  // private CommandXboxController operJoy = new CommandXboxController(Constants.kJoystickPort);

  public Robot() {
    //robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    m_elevator.updateTelemetry();
    //CommandScheduler.getInstance().run();
    
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevator.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    // operJoy.a().onTrue(m_elevator.reachGoalCommand(Constants.kSetpointMetersFirst));
    // operJoy.b().onTrue(m_elevator.reachGoalCommand(Constants.kSetpointMetersSecond));
    // operJoy.x().onTrue(m_elevator.reachGoalCommand(Constants.kSetpointMetersThird));
    // operJoy.y().onTrue(m_elevator.reachGoalCommand(Constants.kSetpointMetersFourth));
   if (m_joystick.getTrigger()) { //used to be m_joystick.getTrigger()
     System.out.println("Goal: " + Constants.kSetpointMetersFirst);
      m_elevator.setMotorVoltage(m_elevator.reachGoal(Constants.kSetpointMetersFirst));
    } 
    else {
      //Otherwise, we update the setpoint to 0.
      System.out.println("released trigger");
      m_elevator.setMotorVoltage(0);
      if(m_elevator.reachGoal(0.0) > 0 ){
        m_elevator.setMotorVoltage(0);
      }
      else{
        m_elevator.setMotorVoltage(m_elevator.reachGoal(0.0));
      }
    }
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  // @Override
  // public void autonomousInit() {

  //   m_autonomousCommand = robotContainer.getAutonomousCommand();
    
  //   // schedule the autonomous command (example)
  //   if (m_autonomousCommand != null) {
  //     m_autonomousCommand.schedule();
  //   }
  // }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
