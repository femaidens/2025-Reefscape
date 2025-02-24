// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Autos;
// import frc.robot.subsystems.Drive;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.Ports.JoyPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
// import frc.robot.commands.Elevating;
// import frc.robot.commands.AlgaeCmds; 
import frc.robot.commands.CoralTransition;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController operJoy = new CommandXboxController(JoyPort.OPERATOR_PORT);
  // private final Climb climb;
  // private final AlgaeCmds algaeCmds = new AlgaeCmds();
  private final AlgaeIntake algaeIntake;
  private final AlgaePivot algaePivot;
  // private final Elevating elevating;
  // private final CoralTransition coralTransition;
  // private final Intake intake; 
  // private final Outtake outtake;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    algaePivot = new AlgaePivot();
    algaeIntake  = new AlgaeIntake();
    // Configure the trigger bindings
    configureBindings();
    // climb = new Climb();
    // elevating = new Elevating();
    // outtake = new Outtake();
    // intake = new Intake();
    // coralTransition = new CoralTransition(intake, outtake);
    configureDefaultCmds();

  }

  private void configureDefaultCmds(){
    // drivetrain.setDefaultCommand(
    //   drivetrain.drive(
    //     () -> MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
    //     () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
    //     () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1))
    //   );
    algaePivot.setDefaultCommand(algaePivot.setGroundCmd());
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

    // xboxController.leftBumper()
    //   .whileTrue(climb.climbFwdCmd());
  

    // xboxController.rightBumper()
    //   .whileTrue(climb.climbBkwdCmd());
    
    // xboxController.rightTrigger()
    //   .whileTrue(climb.pulleySystemCmd());

    //algaeintake

    operJoy.rightBumper()
      .whileTrue(algaeIntake.runRollersCmd())//.alongWith(algaePivot.setGroundCmd()))
      .onFalse(algaeIntake.stopRollersCmd());

    operJoy.leftBumper()
      .whileTrue(algaeIntake.reverseRollersCmd())
      .onFalse(algaeIntake.stopRollersCmd());
    
    operJoy.leftTrigger()
      .whileTrue(algaePivot.setGroundCmd());
    
    operJoy.rightTrigger()
      .whileTrue(algaePivot.setProcessorCmd());
    
    //coralouttake

  //   operJoy.a()
  //     .whileTrue(elevating.firstLevelCmd());
    
  //   operJoy.b()
  //     .whileTrue(elevating.secondLevelCmd());

  //   operJoy.y()
  //     .whileTrue(elevating.thirdLevelCmd());

  //   operJoy.x()
  //     .whileTrue(elevating.fourthLevelCmd());
    
  //   //algaeremoval

  //   operJoy.back()
  //     .whileTrue(elevating.algaeSecondLevelCmd());
    
  //   operJoy.start() 
  //     .whileTrue(elevating.algaeThirdLevelCmd());

  //   //reset default
  //   operJoy.leftTrigger()
  //     .whileTrue(elevating.resetDefault());
    
  //   //transition intake to outtake
  //   operJoy.rightTrigger()
  //     .whileTrue(coralTransition.moveCoralToOuttake());

  //   // run climb spool
  //   operJoy.povUp()
  //     .whileTrue(climb.pulleySystemCmd());
    // operJoy.rightBumper()
    //   .whileTrue(algaeIntake.runRollersCmd())
    //   .onFalse(algaeIntake.stopRollersCmd());

    // operJoy.leftBumper()
    //   .whileTrue(algaeIntake.reverseRollersCmd())
    //   .onFalse(algaeIntake.stopRollersCmd());

    
    
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

