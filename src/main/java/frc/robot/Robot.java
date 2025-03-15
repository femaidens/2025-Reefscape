// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import monologue.Logged;
import monologue.Monologue;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot implements Logged {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private PhotonCamera frontLeftCam;
  //private Drive drive;
  // double forward;
  // double turn;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // forward = 0.0;
    // turn = 0.0;
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //drive = new Drive();

    boolean fileOnly = false;
    boolean lazyLogging = false;
    Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
    // SignalLogger.setPath("/logsNew/");
    SignalLogger.start();
    frontLeftCam = new PhotonCamera("2265-ironfish");

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //isFMSConnected() deprecated
    Monologue.setFileOnly(DriverStation.isFMSAttached());
     // This method needs to be called periodically, or no logging annotations will process properly.
     Monologue.updateAll();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SignalLogger.stop();
  }

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
    SignalLogger.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // double range = 0.0;
    // boolean targetVisible = false;
    //     double targetYaw = 0.0;
    //     double targetRange = 0.0;
    //     var results = frontLeftCam.getAllUnreadResults();
    //     if (!results.isEmpty()) {
    //         // Camera processed a new frame since last
    //         // Get the last one in the list.
    //         var result = results.get(results.size() - 1);
    //         if (result.hasTargets()) {
    //             // At least one AprilTag was seen by the camera
    //             for (var target : result.getTargets()) {
    //                 if (target.getFiducialId() == 7) {
    //                     // Found Tag 7, record its information
    //                     targetYaw = target.getYaw();
    //                     targetRange =
    //                             PhotonUtils.calculateDistanceToTargetMeters(
    //                                     0.5, // Measured with a tape measure, or in CAD.
    //                                     1.435, // From 2024 game manual for ID 7
    //                                     Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
    //                                     Units.degreesToRadians(target.getPitch()));

    //                     targetVisible = true;

    //                     if (targetVisible) {
    //                       // Driver wants auto-alignment to tag 7
    //                       // And, tag 7 is in sight, so we can turn toward it.
    //                       // Override the driver's turn and fwd/rev command with an automatic one
    //                       // That turns toward the tag, and gets the range right.
    //                       turn =
    //                               (-targetYaw) * DriveConstants.Translation.PID.P * DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
    //                       forward =
    //                               (range - targetRange) * DriveConstants.Translation.PID.P * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
    //                   }
    //                 }
    //             }
    //         }
    //     }
    //     drive.drive(() -> forward, () -> .1, () -> turn);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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