// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToCenter;
import frc.robot.commands.Autos;
// import frc.robot.commands.DriveToPoseCmd;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import monologue.Logged;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Seconds;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // The robot's subsystems and commands are defined here...
  // private final Drive drive = new Drive();
  private final Vision vision = new Vision();
  private final Elevator elevator = new Elevator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveJoy = new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operJoy = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  // private final AlignToCenter alignToCenter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // alignToCenter = new AlignToCenter(drive, vision, null);
    configureBindings();
    configureDefaultCmds();
  }

  private void configureDefaultCmds(){
    vision.setDefaultCommand(
      new RunCommand(
        () -> 
        vision.driveFromVision(
        () -> MathUtil.applyDeadband(driveJoy.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(driveJoy.getLeftX(), 0.1),
        () -> MathUtil.applyDeadband(driveJoy.getRightX(), 0.1)),
        vision));
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
    driveJoy.rightBumper()
    .onTrue(vision.resetGyroFromVision());
    
    // operJoy.a()
    // .onTrue(new DriveToPoseCmd(drive, vision::getCurrentPose));

//     operJoy.b()
//     .onTrue(vision.printYaw())
//     .onFalse(vision.stopDriving());
    
//     operJoy.x()
//     .onTrue(vision.driveTranslational())
//     .onFalse(vision.stopDriving());

//     operJoy.y()
//     .onTrue(vision.run(() -> vision.funky()))
//     .onFalse(vision.stopDriving());

    operJoy.rightTrigger()
    .onTrue(vision.funkierRight())
    .onFalse(vision.stopDriving());

    operJoy.leftTrigger()
    .onTrue(vision.funkierLeft())
    .onFalse(vision.stopDriving());

     operJoy.b()
                .onTrue(
                elevator.setLevel(ElevatorConstants.SetpointConstants.SECOND_LVL).until(elevator::atSetpoint).andThen(elevator.stopMotorCmd()));
                //.onFalse(elevator.stopMotorCmd());

                operJoy.y()
                .onTrue(
                elevator.setLevel(ElevatorConstants.SetpointConstants.THIRD_LVL).until(elevator::atSetpoint).andThen(elevator.stopMotorCmd()));

                operJoy.x()
                .onTrue(
                elevator.setLevel(ElevatorConstants.SetpointConstants.FOURTH_LVL).until(elevator::atSetpoint).andThen(elevator.stopMotorCmd()));

                operJoy.povUp()
                .whileTrue(elevator.runMotorCmd())
                .onFalse(elevator.stopMotorCmd());

                operJoy.povDown()
                .whileTrue(elevator.forceReverseMotorCmd())
                .onFalse(elevator.stopMotorCmd());

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
