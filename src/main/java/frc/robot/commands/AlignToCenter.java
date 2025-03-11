// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveConstants;
import frc.robot.Constants.*;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCenter extends Command {
  private Drive drive;
  private Vision vision;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private PhotonTrackedTarget target;
  public AlignToCenter(Drive drive, Vision vision, PhotonTrackedTarget target) {
    this.drive = drive;
    this.target = vision.getTag();
    xController = new PIDController(DriveConstants.Translation.PID.P, DriveConstants.Translation.PID.I, DriveConstants.Translation.PID.D);

    yController = new PIDController(DriveConstants.Translation.PID.P, DriveConstants.Translation.PID.I, DriveConstants.Translation.PID.D);

    thetaController = new PIDController(DriveConstants.Turn.PID.P, DriveConstants.Turn.PID.I, DriveConstants.Turn.PID.D);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }
  //change to command

  public Command driveToCmd(Pose2d target){
    System.out.println("running");
    return new RunCommand(
      () -> drive.drive(
        () -> xController.calculate(target.getTranslation().getX())*DriveConstants.Translation.PID.P*DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond),
        () -> yController.calculate(target.getTranslation().getY()),
        () -> thetaController.calculate(target.getRotation().getRadians())*DriveConstants.Turn.PID.P*DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)),
      drive);
  }

  // public Command align() {
  //   new RunCommand(
  //   () -> drive.drive(
  //   //need to change 1 to desired range from apriltag
  // () -> (1-vision.distanceToTarget(target)*DriveConstants.Translation.PID.P*DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond)), 
  // () -> 0, 
  // () -> target.getYaw()*DriveConstants.Turn.PID.P*DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)), drive);
  // }
}
