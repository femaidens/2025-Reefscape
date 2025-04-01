// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.TaxiL2;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;
//import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveSim;

import java.util.function.DoubleSupplier;

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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.ModuleConfig;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Drive drivetrain = new Drive();
  DriveSim driveSim = new DriveSim();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveJoy = new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operJoy = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private SendableChooser<Command> autoChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCmds();
    // configureAuton();
    // ModuleConfig moduleConfig = new ModuleConfig(0.5, 15, 0.1, DCMotor.getKrakenX60(1), DriveConstants.Translation.CURRENT_LIMIT, 1);
    // RobotConfig config = new RobotConfig(60, 1.0/6*60*Drivetrain.TRACK_WIDTH, moduleConfig, Drivetrain.TRACK_WIDTH);
    
    // AutoBuilder autoBuilder = new AutoBuilder();
        // AutoBuilder.configure(
        // driveSim::getPose, 
        // driveSim::resetOdometry, 
        // driveSim::getRobotRelativeChassisSpeeds, //chassis speed supplier must be robot relative
        // driveSim::setChassisSpeeds, //method that will drive the robot based on robot relative chassis speed
        // driveSim.holonomicDriveController, 
        // config,  

        // () -> {
        // var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        // driveSim);
    
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<>();

    // boolean isCompetition = true;

    // // Build an auto chooser. This will use Commands.none() as the default option.
    // // As an example, this will only show autos that start with "comp" while at
    // // competition as defined by the programmer
    // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    //   (stream) -> isCompetition
    //     ? stream.filter(auto -> auto.getName().startsWith("Start"))
    //     : stream
    // );
    // SmartDashboard.putData("Choose Auto: ", autoChooser);
    configureAuto();
  }

  private void configureDefaultCmds(){
    // drivetrain.setDefaultCommand(
    //   drivetrain.drive(
    //     () -> MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1),
    //     () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
    //     () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1))
    //   );
    driveSim.setDefaultCommand(
      driveSim.drive(()-> -driveJoy.getLeftY(), ()-> -driveJoy.getLeftX(), () ->-driveJoy.getRightX()));
    
  }

  public void configureAuto(){
    autoChooser.addOption("taxi l2", new TaxiL2(driveSim));
    SmartDashboard.putData("Choose Auto", autoChooser);
  }

   public void configureAuton() {
    // ModuleConfig moduleConfig = new ModuleConfig(0.5, 15, 0.1, DCMotor.getKrakenX60(1), DriveConstants.Translation.CURRENT_LIMIT, 1);
    // RobotConfig config = new RobotConfig(60, 1.0/6*60*Drivetrain.TRACK_WIDTH, moduleConfig, Drivetrain.TRACK_WIDTH);
    
    // // AutoBuilder autoBuilder = new AutoBuilder();
    //     AutoBuilder.configure(
    //     driveSim::getPose, 
    //     driveSim::resetOdometry, 
    //     driveSim::getRobotRelativeChassisSpeeds, //chassis speed supplier must be robot relative
    //     driveSim::setChassisSpeeds, //method that will drive the robot based on robot relative chassis speed
    //     driveSim.holonomicDriveController, 
    //     config,  

    //     () -> {
    //     var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //     driveSim);
// new HolonomicPathFollowerConfig(
        //     new PIDConstants(Translation.PID.P, Translation.PID.I, Translation.PID.D), // Translation PID constants
        //     new PIDConstants(Turn.PID.P, Turn.PID.I, Turn.PID.D), // Rotation PID constants
        //     DriveConstants.Drivetrain.MAX_SPEED, // Max module speed, in m/s
        //     1.0/Math.PI, 
        //     new ReplanningConfig()),
  }

  
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    // try{
    //     // Load the path you want to follow using its name in the GUI
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("Start center");
    //     System.out.println("Path set!");

    //     // Create a path following command using AutoBuilder. This will also trigger event markers.
    //     return AutoBuilder.followPath(path);
    // } catch (Exception e) {
    //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //     return Commands.none();
    // }
    System.out.println("autonomous run!!!");
    return autoChooser.getSelected();
  }
}
