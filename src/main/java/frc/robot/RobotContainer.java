// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drive;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // The robot's subsystems and commands are defined here...
  Drive drivetrain = new Drive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveJoy = new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operJoy = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCmds();
  }

  private void configureDefaultCmds(){
    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> 
        drivetrain.drive(
        () -> MathUtil.applyDeadband(driveJoy.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
        () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1)),
        drivetrain));
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
    driveJoy.a()
    .whileTrue(
        drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward)
    );

    driveJoy.b()
    .whileTrue(
        drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse)
    );

    driveJoy.x()
    .whileTrue(
        drivetrain.driveDynamic(SysIdRoutine.Direction.kForward)
    );

    driveJoy.y()
    .whileTrue(
        drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse)
    );

    driveJoy.leftBumper()
    .whileTrue(
      drivetrain.setXCmd()
    );

    driveJoy.rightBumper()
    .whileTrue(
      drivetrain.resetGyro()
    );

    driveJoy.leftTrigger()
    .whileTrue(
      drivetrain.setStraightCmd()
    );

    driveJoy.rightTrigger()
    .whileTrue(
      drivetrain.driveStraightCmd()
    );
  }



  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
