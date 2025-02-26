// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.SignalLogger;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import monologue.Annotations.Log;
import monologue.Logged;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;


public class Drive extends SubsystemBase implements Logged {

  private final ModuleKraken frontLeft;
  private final ModuleKraken frontRight;
  private final ModuleKraken rearLeft;
  private final ModuleKraken rearRight;

  private final List<ModuleKraken> modules;

  private final AHRS gyro;

  private final SwerveDriveOdometry odometry;

  private final SysIdRoutine driveRoutine;

  private final SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new Drive. */
  public Drive() {
    // frontLeft = new ModuleSpark(DrivetrainPorts.FRONT_LEFT_DRIVE,
    // DrivetrainPorts.FRONT_LEFT_TURN, Translation.FRONT_LEFT_ANGOFFSET);
    // frontRight = new ModuleSpark(DrivetrainPorts.FRONT_RIGHT_DRIVE,
    // DrivetrainPorts.FRONT_RIGHT_TURN, Translation.FRONT_RIGHT_ANGOFFSET);
    // rearLeft = new ModuleSpark(DrivetrainPorts.REAR_LEFT_DRIVE,
    // DrivetrainPorts.REAR_LEFT_TURN, Translation.REAR_LEFT_ANGOFFSET);
    // rearRight = new ModuleSpark(DrivetrainPorts.REAR_RIGHT_DRIVE,
    // DrivetrainPorts.REAR_RIGHT_TURN, Translation.REAR_RIGHT_ANGOFFSET);
    frontLeft = new ModuleKraken(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN,
        DrivetrainPorts.FRONT_LEFT_CANCODER, Translation.FRONT_LEFT_MAG_OFFSET, Translation.FRONT_LEFT_ANGOFFSET, true);
    frontRight = new ModuleKraken(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN,
        DrivetrainPorts.FRONT_RIGHT_CANCODER, Translation.FRONT_RIGHT_MAG_OFFSET, Translation.FRONT_RIGHT_ANGOFFSET,
        true);
    rearLeft = new ModuleKraken(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN,
        DrivetrainPorts.REAR_LEFT_CANCODER, Translation.REAR_LEFT_MAG_OFFSET, Translation.REAR_LEFT_ANGOFFSET, true);
    rearRight = new ModuleKraken(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN,
        DrivetrainPorts.REAR_RIGHT_CANCODER, Translation.REAR_RIGHT_MAG_OFFSET, Translation.REAR_RIGHT_ANGOFFSET, true);

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

    // totally not sure, would need to check
    gyro = new AHRS(NavXComType.kMXP_UART);

    odometry = new SwerveDriveOdometry(
        Drivetrain.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition()
        });
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator =
      new SwerveDrivePoseEstimator(
        Drivetrain.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(),
          frontRight.getSwerveModulePosition(),
          rearLeft.getSwerveModulePosition(),
          rearRight.getSwerveModulePosition()
      },
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    zeroHeading();

    driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Use default timeout (10 s)
                  // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> modules.forEach(m -> m.setDriveVoltage(volts.in(Units.Volts))),
            null,
            this));
    // driveRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
    // volts -> modules.forEach(m -> m.setDriveVoltage(volts.in(Units.Volts))),
    // null, this));

    // SmartDashboard.putNumber("Gyro angle", gyro.getRotation2d().getDegrees());
  }

  // consider changing to profiledpid control
  /**
   * drivin
   * 
   * @param xSpeed   x direction (front and back)
   * @param ySpeed   y direction (right is positive, left is negative)
   * @param rotSpeed
   * @return
   */
  public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates);
  }

  /**
   * sets the swerve ModuleStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR);
    frontLeft.setDesiredStateNoPID(desiredStates[1]); // frontLeft.setDesiredStateNoPID(desiredStates[1]);
    frontRight.setDesiredStateNoPID(desiredStates[0]); // frontRight.setDesiredStateNoPID(desiredStates[0]);
    rearLeft.setDesiredStateNoPID(desiredStates[3]); // rearLeft.setDesiredStateNoPID(desiredStates[3]);
    rearRight.setDesiredStateNoPID(desiredStates[2]); // rearRight.setDesiredStateNoPID(desiredStates[2]);
  }

  /**
   * x formation with wheels to prevent movement
   */
  public Command setXCmd() {
    return this.run(
        () -> {
          frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
          frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
          rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
          rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
        });
  }

  /**
   * Sets wheels straight for sysid
   */
  public Command setStraightCmd() {
    return this.run(
        () -> {
          frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
          frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
          rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
          // only rear right is acting up, consider changing it to 180 degrees
          rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        });
  }

  public Command driveStraightCmd() {
    return this.run(
        () -> {
          frontLeft.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
          frontRight.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
          rearLeft.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
          rearRight.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
        });
  }

  /**
   * @return currently-estimated pose of robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Log.NT
  public SwerveModuleState[] getSwerveModuleStates() {
    return modules.stream().map(m -> m.getState()).toArray(SwerveModuleState[]::new);
  }

  @Log.NT
  public SwerveModuleState[] getDesiredSwerveModuleStates() {
    return modules.stream().map(m -> m.getDesiredState()).toArray(SwerveModuleState[]::new);
  }

  /**
   * resets the odometry to the specified pose
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromRadians(gyro.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition() },
        pose);
  }

  /**
   * Gets the angle of the gyro in radians (ideally)
   * 
   * @return in radians
   */
  @Log.NT
  public double getAngle() {
    return -1 * gyro.getAngle();
  }

  /**
   * In case we'll ever need it
   * 
   * @return new yaw angle in radians (ideally)
   */
  public double setYawOffset() {
    gyro.setAngleAdjustment(-Math.PI / 2); // need to double check!
    return gyro.getYaw();
  }

  /**
   * Zero the gyro heading
   */
  public void zeroHeading() {
    gyro.reset();
  }

  public Command resetGyro() {
    return this.run(
        () -> zeroHeading());
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
  // public boolean isFacing(Translation2d target) {
  //     return Math.abs(
  //             gyro.getRotation2d().getRadians()
  //                 - target.minus(poseEstimator.getEstimatedPosition().getTranslation()).getAngle().getRadians()) < turnPIDController.getErrorTolerance();
  // }

  // public Command driveTo(Pose2d target) {
  //   return run() -> {
  //         Transform2d transform = poseEstimator.getEstimatedPosition().minus(target);
  //         Vector<N3> difference =
  //             VecBuilder.fill(
  //                 transform.getX(),
  //                 transform.getY(),
  //                 transform.getRotation().getRadians());
  //         double out = drivePIDController.calculate(difference.norm(), 0);
  //         Vector<N3> velocities = difference.unit().times(out);
  //         setChassisSpeeds(
  //             new ChassisSpeeds();
  //       })

  // public boolean isFacing(Translation2d target) {
  //   var results = camera.getAllUnreadResults();
  //       if (!results.isEmpty()) {
  //           // Camera processed a new frame since last
  //           // Get the last one in the list.
  //           var result = results.get(results.size() - 1);
  //           if (result.hasTargets()) {
  //               // At least one AprilTag was seen by the camera
  //               for (var target : result.getTargets()) {
  //                   if (target.getFiducialId() == 7) {
  //                       // Found Tag 7, record its information
  //                       targetYaw = target.getYaw();
  //                       targetVisible = true;
  //                   }
  //               }
  //           }
  //       }
  //   if (controller.getAButton() && targetVisible) {
  //           // Driver wants auto-alignment to tag 7
  //           // And, tag 7 is in sight, so we can turn toward it.
  //           // Override the driver's turn command with an automatic one that turns toward the tag.
  //           turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
  //       }
  // }
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond));
    setModuleStates(
        Drivetrain.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
              Drivetrain.kDriveKinematics.toChassisSpeeds(states), Seconds.of(0.02))));
  }

  /* SYSID CMDS */
  public Command driveQuasistatic(SysIdRoutine.Direction direction) {
    return driveRoutine.quasistatic(direction);
  }

  public Command driveDynamic(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if gyro is inverted, getRotation2d() --- getAngle() can be negated
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()
        });
    poseEstimator.update(gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(),
        rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()
        });

    SmartDashboard.updateValues();
    SmartDashboard.putNumber("angle", getAngle());
  }
}