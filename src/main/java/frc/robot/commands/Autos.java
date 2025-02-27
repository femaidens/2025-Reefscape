// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public final class Autos {

  private final Drive drivetrain;
  private final Outtake outtake;
  private final Intake intake;
  private RobotConfig config;
  private SendableChooser<Command> autonChooser;


  private Autos(Drive drive, Outtake outtake, Intake intake, Elevator elevator, AlgaeIntake algaeIntake) {

    autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    config = new RobotConfig(
      Constants.PathPlannerConstants.massKg, 
      Constants.PathPlannerConstants.MOI,
      new ModuleConfig(
        DriveConstants.Translation.WHEEL_RADIUS, 
        DriveConstants.Drivetrain.MAX_SPEED, 
        DriveConstants.Drivetrain.WHEEL_COF, 
        DCMotor.getKrakenX60(8), 
        DriveConstants.Translation.CURRENT_LIMIT, 
        8), 
        DriveConstants.Drivetrain.TRACK_WIDTH);


    this.intake = intake;
    this.outtake = outtake;
    this.drivetrain = drive;

  }

  public boolean shouldWeFlip(){
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
  }

  public SendableChooser<Command> configure() {
    try{
      config = RobotConfig.fromGUISettings();
    }catch(
    Exception e)
    {
    // Handle exception as needed
    e.printStackTrace();
    }

    AutoBuilder.configure(
      drivetrain::getPose, 
      drivetrain::resetOdometry,
      drivetrain::getCurrentChassisSpeeds,
      (s, feedforwards) -> drivetrain.setChassisSpeeds(s),
      new PPHolonomicDriveController(
        new PIDConstants(Translation.PID.P, Translation.PID.D), 
        new PIDConstants(Turn.PID.P, Turn.PID.D)),
      config,
      () -> shouldWeFlip(),
      drivetrain);

      NamedCommands.registerCommand("outtake trough", outtake.setOuttakeCoralSpeedCmd());
      NamedCommands.registerCommand("intake", intake.intakeCoralCmd());
  }
}
