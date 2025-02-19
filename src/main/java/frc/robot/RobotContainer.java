// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralTransition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.Command;
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
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController operJoy =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Outtake outtake;
  private final Intake intake;

  private final CoralTransition coralTransition;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    outtake = new Outtake();
    intake = new Intake();
    coralTransition = new CoralTransition(intake, outtake);
    // Configure the trigger bindings

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    operJoy.a()
     .whileTrue(outtake.setIntakeCoralSpeedCmd())
     .onFalse(outtake.stopMotorCmd()); 

     operJoy.b()
     .whileTrue(outtake.setOuttakeCoralSpeedCmd())
     .onFalse(outtake.stopMotorCmd());

     operJoy.x()
     .whileTrue(outtake.removeAlgaeCmd())
     .onFalse(outtake.stopMotorCmd());

     operJoy.y()
     .whileTrue(outtake.setVoltageCmd())
     .onFalse(outtake.stopMotorCmd());

     operJoy.leftBumper()
     .whileTrue(coralTransition.moveCoralToOuttake())
     .onFalse(outtake.stopMotorCmd());

  




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
