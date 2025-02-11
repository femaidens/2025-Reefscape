// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.ElevatorCommands;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GeneralElevator;;

public class RobotContainer {

  private CommandXboxController operJoy = new CommandXboxController(Constants.kJoystickPort);

  private final GeneralElevator elevator = new GeneralElevator();

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
    // autonChooser.addOption("Stage 1", new ElevatorCommands(elevator).stage1Command());
    // autonChooser.addOption("Stage 2", new ElevatorCommands(elevator).stage2Command());
    // autonChooser.addOption("Stage 3", new ElevatorCommands(elevator).stage3Command());
    // autonChooser.addOption("Stage 4", new ElevatorCommands(elevator).stage4Command());
  }

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
    return autonChooser.getSelected();
  }
}