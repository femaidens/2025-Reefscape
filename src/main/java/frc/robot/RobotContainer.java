// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Ports.JoyPort;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.AlgaeIntake;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xboxController;
  // private final CommandXboxController operJoy = new CommandXboxController(JoyPort.OPERATOR_PORT);
  // private final Climb climb;
  private final LED leds;
  private final AlgaeIntake algaeIntake = new AlgaeIntake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    xboxController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    leds = new LED();
    configureBindings();
    // climb = new Climb();
    configureDefaultCmds();
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

   private void configureDefaultCmds(){
    leds.setDefaultCmd(leds.setPurpleCmd());
  }

  private void configureBindings() {


  xboxController.rightBumper()
    .onTrue(leds.setGreenCmd().withTimeout(3));
      //algaeIntake.runRollersCmd()
      //.andThen(leds.setGreenCmd().withTimeout(3))
    
    
  // xboxController.rightBumper()
  //   .onFalse(algaeIntake.stopRollersCmd());
    

  xboxController.leftBumper()
    .onTrue(leds.setPurpleCmd().withTimeout(3));
      //algaeIntake.reverseRollersCmd()
      //.andThen(leds.setGreenCmd().withTimeout(2))
    

  // xboxController.rightBumper()
  //   .onFalse(algaeIntake.stopRollersCmd());

  // climb commands
    // xboxController.leftBumper()
    //   .whileTrue(climb.climbFwdCmd());
  

    // xboxController.rightBumper()
    //   .whileTrue(climb.climbBkwdCmd());
    
    // xboxController.rightTrigger()
    //   .whileTrue(climb.pulleySystemCmd());
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

