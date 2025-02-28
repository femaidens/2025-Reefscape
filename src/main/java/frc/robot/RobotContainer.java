// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;
//import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GeneralElevator;

import java.util.GregorianCalendar;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private GeneralElevator elevator = GeneralElevator.create();
  private Drive drive = new Drive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.DRIVER_PORT);

  private final CommandXboxController operJoy = 
      new CommandXboxController(ControllerConstants.OPERATOR_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // configurations
    configureButtonBindings();
//     configureAuton();
    configureDefaultCommands();
  }

  public void configureSubsystemDefaults() {
  }

  public void configureDefaultCommands() {
  }
  
//   public void configureAuton() {
//     SmartDashboard.putData("Choose Auto: ", autonChooser);
//     // autonChooser.addOption("Stage 1", new ElevatorCommands(elevator).stage1Command());
//     // autonChooser.addOption("Stage 2", new ElevatorCommands(elevator).stage2Command());
//     // autonChooser.addOption("Stage 3", new ElevatorCommands(elevator).stage3Command());
//     // autonChooser.addOption("Stage 4", new ElevatorCommands(elevator).stage4Command());
//   }

  private void configureButtonBindings() {
        // stage 1
        operJoy.a()
            .onTrue(elevator.reachGoal(Constants.ElevatorConstants.SetpointConstants.FIRST_LVL)
            );
            
        // stage 2
        operJoy.b()
            .onTrue(elevator.reachGoal(Constants.ElevatorConstants.SetpointConstants.SECOND_LVL)
            );

        // stage 3
        operJoy.y()
            .onTrue(elevator.reachGoal(Constants.ElevatorConstants.SetpointConstants.THIRD_LVL)
            );

        // stage 4
        operJoy.x()
            .onTrue(elevator.reachGoal(Constants.ElevatorConstants.SetpointConstants.FOURTH_LVL)
            );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Start center");
        System.out.println("Path set!");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}