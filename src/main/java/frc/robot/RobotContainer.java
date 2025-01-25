// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Elevator;


public class RobotContainer {

  private CommandXboxController operJoy = new CommandXboxController(Constants.kJoystickPort);

  private final Elevator elevator = new Elevator();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    // configurations
    configureButtonBindings();
    configureAuton();
    configureDefaultCommands();
  }

  public void configureSubsystemDefaults() {
  }

  public void configureDefaultCommands() {
  }
  
  public void configureAuton() {
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.addOption("Stage 1", new ElevatorCommands(elevator).stage1Command());
    autonChooser.addOption("Stage 2", new ElevatorCommands(elevator).stage2Command());
    autonChooser.addOption("Stage 3", new ElevatorCommands(elevator).stage3Command());
    autonChooser.addOption("Stage 4", new ElevatorCommands(elevator).stage4Command());
  }

  private void configureButtonBindings() {
        // stage 1
        operJoy.a()
            .onTrue(new ElevatorCommands(elevator).stage1Command()
            );
            
        // stage 2
        operJoy.b()
            .onTrue(new ElevatorCommands(elevator).stage2Command()
            );

        // stage 3
        operJoy.y()
            .onTrue(new ElevatorCommands(elevator).stage3Command()
            );

        // stage 4
        operJoy.x()
            .onTrue(new ElevatorCommands(elevator).stage4Command()
            );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}