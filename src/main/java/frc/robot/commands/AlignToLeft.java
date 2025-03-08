// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveConstants;
import frc.robot.Constants.*;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToLeft extends Command {
  private Drive drive;
  private Vision vision;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private PhotonTrackedTarget target;
  public AlignToLeft(Drive drive, Vision vision, PhotonTrackedTarget target) {
    this.drive = drive;
    this.target = target;
    xController = new PIDController(DriveConstants.Translation.PID.P, DriveConstants.Translation.PID.I, DriveConstants.Translation.PID.D);

    yController = new PIDController(DriveConstants.Translation.PID.P, DriveConstants.Translation.PID.I, DriveConstants.Translation.PID.D);

    thetaController = new PIDController(DriveConstants.Turn.PID.P, DriveConstants.Turn.PID.I, DriveConstants.Turn.PID.D);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(vision.getTagArea(target) > 0.1){
      drive.drive(() -> 0.1, () -> 0, () -> 0);
    }else{
      drive.drive(() -> 0, () -> 0, () -> 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(() -> 0, () -> 0, () -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
