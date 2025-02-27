// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Autos;
// import frc.robot.subsystems.Drive;
import monologue.Logged;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
<<<<<<< HEAD
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
>>>>>>> main
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.subsystems.Climb; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
=======
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.subsystems.Climb; 
>>>>>>> main

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // The robot's subsystems and commands are defined here...
  // Drive drive = new Drive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveJoy = new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operJoy = new CommandXboxController(OperatorConstants.OPERATOR_PORT);
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final AlgaePivot algaePivot = new AlgaePivot();
  private final Drive drivetrain = new Drive();
  private RobotConfig config;
  
    private SendableChooser<Command> autonChooser;
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      // Configure the trigger bindings
      
      configureDefaultCmds();
      configureBindings();
    }
  
    private void configureDefaultCmds() {
      drivetrain.setDefaultCommand(
          new RunCommand(
              () -> drivetrain.drive(
                  () -> MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
                  () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
                  () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1))));
                  
      // drive.setDefaultCommand(
      // drive.drive(
      // () -> MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
      // () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
      // () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1))
      // );
  
      algaePivot.setDefaultCommand(algaePivot.setProcessorCmd());
      }
  
  
    
  
  
//     public void configureAuton(){
      

// //  // Configure AutoBuilder last
// //     AutoBuilder.configure(drivetrain.getPose(), // Robot pose supplier
// //     drivetrain.resetOdometry(drivetrain.getPose()), // Method to reset odometry (will be called if your auto has a starting pose)
// //     drivetrain.getChassisSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
// //     (speeds,feedforwards)->

// //     drivetrain.autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
// //               new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
// //                       new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
// //                       new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
// //               ),
// //               config, // The robot configuration
// //               () -> {
// //                 // Boolean supplier that controls when the path will be mirrored for the red alliance
// //                 // This will flip the path being followed to the red side of the field.
// //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

// //                 var alliance = DriverStation.getAlliance();
// //                 if (alliance.isPresent()) {
// //                   return alliance.get() == DriverStation.Alliance.Red;
// //                 }
// //                 return false;
// //               },
// //               drivetrain // Reference to this subsystem to set requirements
// //       ),

//     // AutoBuilder.configure(null, null, null, null, null, config, null, null);
    
//   }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveJoy.a()
        .whileTrue(
            drivetrain.driveQuasistatic(SysIdRoutine.Direction.kForward));

    driveJoy.b()
        .whileTrue(
            drivetrain.driveQuasistatic(SysIdRoutine.Direction.kReverse));

    driveJoy.x()
        .whileTrue(
            drivetrain.driveDynamic(SysIdRoutine.Direction.kForward));

    driveJoy.y()
        .whileTrue(
            drivetrain.driveDynamic(SysIdRoutine.Direction.kReverse));

    driveJoy.leftBumper()
        .whileTrue(
            drivetrain.setXCmd());

    driveJoy.rightBumper()
        .whileTrue(
            drivetrain.resetGyro());

    driveJoy.leftTrigger()
        .whileTrue(
            drivetrain.setStraightCmd());

    driveJoy.rightTrigger()
        .whileTrue(
            drivetrain.driveStraightCmd());
  }

=======
 
>>>>>>> main
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */

  // public void configureAuton(){
  // AutoBuilder.configureHolonomic(
  // drive::getPose,
  // drive::resetOdometry,
  // drive::getRobotRelativeChassisSpeeds, //chassis speed supplier must be robot
  // relative
  // drive::setChassisSpeeds, //method that will drive the robot based on robot
  // relative chassis speed
  // new HolonomicPathFollowerConfig(
  // new com.pathplanner.lib.config.PIDConstants(Drive.kP, Drive.kI, Drive.kD), //
  // Translation PID constants
  // new com.pathplanner.lib.config.PIDConstants(Turning.kP, Turning.kI,
  // Turning.kD), // Rotation PID constants
  // DriveConstants.MAX_SPEED, // Max module speed, in m/s
  // ModuleConstants.WHEEL_DIAMETER/2,
  // new ReplanningConfig()),
  // () -> {
  // var alliance = DriverStation.getAlliance();
  // if (alliance.isPresent()) {
  // return alliance.get() == DriverStation.Alliance.Red;
  // }
  // return false;
  // },
  // drive);

  // }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

}
