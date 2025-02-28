// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Elevator;
import monologue.Logged;
import monologue.Monologue;
import frc.robot.subsystems.*;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
// import monologue.Monologue; 
// import monologue.Logged; 


/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot implements Logged{
  // private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);
  private final Elevator m_elevator = new Elevator();
  private Command m_autonomousCommand;
  // private DriveSim bobot = new DriveSim();
  // private final Joystick m_joystick = new Joystick(Constants.OperatorConstants.kJoystickPort);
  RobotContainer m_RobotContainer;

  private RobotContainer m_robotContainer;
  // private CommandXboxController operJoy = new CommandXboxController(Constants.kJoystickPort);

  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // If publishing to NetworkTables and DataLog
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    URCL.start();

    // If logging only to DataLog
    // URCL.start(DataLogManager.getLog());
  }

  @Override
  public void robotInit() {
      boolean fileOnly = false;
      boolean lazyLogging = true;
      Monologue.setupMonologue(this, "/Robot", false, true);
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    m_elevator.updateTelemetry();
    //CommandScheduler.getInstance().run();
    
     // setFileOnly is used to shut off NetworkTables broadcasting for most logging calls.
     // Basing this condition on the connected state of the FMS is a suggestion only.
    //  Monologue.setFileOnly(DriverStation.isFMSAttached());
     // This method needs to be called periodically, or no logging annotations will process properly.
     Monologue.updateAll();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  //@Override
  // public void teleopPeriodic() {
    
  //   // if (m_joystick.getTrigger()) {
  //   //   // Here, we set the constant setpoint of 0.75 meters.
  //   //   bobot.driveForward();

  //   // } else {
  //   //   // Otherwise, we update the setpoint to 0.
  //   //   bobot.close();

  //   // }
  // }

  //@Override
  // public void testInit() {
  //   // Cancels all running commands at the start of test mode.
  //   CommandScheduler.getInstance().cancelAll();
  // }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
