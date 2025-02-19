// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
// import frc.robot.subsystems.Drive;
import frc.robot.subsystems.AlgaeIntake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climb; 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Drive drivetrain = new Drive();

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
    // drivetrain.setDefaultCommand(
    //   drivetrain.drive(
    //     () -> MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
    //     () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
    //     () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1))
    //   );
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
//     operJoy.rightBumper()
//       .whileTrue(algaeIntake.runRollersCmd())
//       .onFalse(algaeIntake.stopRollersCmd());

//     operJoy.leftBumper()
//       .whileTrue(algaeIntake.reverseRollersCmd())
//       .onFalse(algaeIntake.stopRollersCmd());

//     operJoy.leftBumper()
//       .whileTrue(climb.climbFwdCmd());
  
//     operJoy.rightBumper()
//       .whileTrue(climb.climbBkwdCmd());
    
//     operJoy.rightTrigger()
//       .whileTrue(climb.pulleySystemCmd());   
  }

 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }}

