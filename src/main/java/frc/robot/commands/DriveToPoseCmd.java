// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseCmd extends Command {

  private static final Distance TRANSLATION_TOLERANCE = Inches.of(1.0);
  private static final Angle THETA_TOLERANCE = Degrees.of(1.0);

  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
    DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond),
    DriveConstants.Translation.MAX_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
    DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
    DriveConstants.Turn.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final Drive drive;
  protected final Supplier<Pose2d> poseProvider;

  /**
   * Constructs a DriveToPoseCommand
   * 
   * @param drive drivetrain subsystem
   * @param goalPose goal pose to drive to
   */
  public DriveToPoseCmd(Drive drive, Supplier<Pose2d> poseProvider) {
    this(drive, poseProvider, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS);
  }

  /**
   * Constructs a DriveToPoseCommand with specific motion profile constraints
   * 
   * @param drive drivetrain subsystem
   * @param poseProvider provider to call to get the robot pose
   * @param translationConstraints translation motion profile constraints
   * @param omegaConstraints rotation motion profile constraints
   */
  public DriveToPoseCmd(
    Drive drive,
      Supplier<Pose2d> poseProvider,
      TrapezoidProfile.Constraints translationConstraints,
      TrapezoidProfile.Constraints omegaConstraints) {

    this.drive = drive;
    this.poseProvider = poseProvider;

    xController = new ProfiledPIDController(DriveConstants.Translation.PID.P, DriveConstants.Translation.PID.I, DriveConstants.Translation.PID.D, translationConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    yController = new ProfiledPIDController(DriveConstants.Translation.PID.P, DriveConstants.Translation.PID.I, DriveConstants.Translation.PID.D, translationConstraints);
    yController.setTolerance(TRANSLATION_TOLERANCE.in(Meters));

    thetaController = new ProfiledPIDController(DriveConstants.Turn.PID.P, DriveConstants.Turn.PID.I, DriveConstants.Turn.PID.D, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE.in(Radians));

    addRequirements(drive);
  }

  /**
   * Sets the goal to drive to. This should be set before the command is scheduled.
   * 
   * @param goalPose goal pose
   */
  public void setGoal(Pose2d goalPose) {
    thetaController.setGoal(goalPose.getRotation().getRadians());
    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
  }

  @Override
  public void initialize() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();

    double xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    double ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    double rotSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      rotSpeed = 0;
    }

    drive.drive(xSpeed, ySpeed, rotSpeed);
  }

  public void resetPose(Pose2d pose) {
    xController.reset(pose.getX());
    yController.reset(pose.getY());
    thetaController.reset(pose.getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
