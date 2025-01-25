// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Vector;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;

public class Drive extends SubsystemBase {

  private final ModuleSpark frontLeft;
  private final ModuleSpark frontRight;
  private final ModuleSpark rearLeft;
  private final ModuleSpark rearRight;

  private final List<ModuleSpark> modules;

  private final AHRS gyro;

  private final SwerveDriveOdometry odometry;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final SysIdRoutine driveRoutine;

  /** Creates a new Drive. */
  public Drive() {
    frontLeft = new ModuleSpark(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN, Translation.FRONT_LEFT_ANGOFFSET);
    frontRight = new ModuleSpark(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN, Translation.FRONT_RIGHT_ANGOFFSET);
    rearLeft = new ModuleSpark(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN, Translation.REAR_LEFT_ANGOFFSET);
    rearRight = new ModuleSpark(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN, Translation.REAR_RIGHT_ANGOFFSET);
    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

    // totally not sure, would need to check
    gyro = new AHRS(NavXComType.kI2C); 

    odometry = new SwerveDriveOdometry(
      Drivetrain.kDriveKinematics, 
      gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        rearLeft.getSwerveModulePosition(),
        rearRight.getSwerveModulePosition()
      });
      zeroHeading();
      var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
      var visionStdDevs = VecBuilder.fill(1, 1, 1);

      poseEstimator = new SwerveDrivePoseEstimator(
        Drivetrain.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[]{
          frontLeft.getSwerveModulePosition(),
          frontRight.getSwerveModulePosition(),
          rearLeft.getSwerveModulePosition(),
          rearRight.getSwerveModulePosition()},
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

      driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
          volts -> modules.forEach(m -> m.setDriveVoltage(volts.in(Units.Volts))), null, this));

      SmartDashboard.putNumber("Gyro angle", gyro.getRotation2d().getDegrees());
    }

  //PLEASE CHECK THE SPEED FACTOR
  //consider changing to profiledpid control
  /**
   * drivin
   * @param xSpeed x direction (front and back)
   * @param ySpeed y direction (right is positive, left is negative)
   * @param rotSpeed 
   * @return
   */
  public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed){
    double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    return this.run(
      () -> {
        for(int i = 0; i < modules.size(); i++){
          modules.get(i).setDesiredState(moduleStates[i]);
        }
      }
    );
  }
  
  /**
   * Gets the angle of the gyro in radians (ideally)
   * @return in radians
   */
  public double getAngle(){
    return -1 * gyro.getAngle();
  }

  /**
   * @return currently-estimated pose of robot
   */
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  // public Command driveTo(Pose2d target){
  //   return run(()-> {
  //     Transform2d transform = getPose().minus(target);
  //     Vector<N3> difference = VecBuilder.fill(
  //       transform.getX(),
  //       transform.getY(),
  //       transform.getRotation().getRadians());
  //   }
  // }
//   public Pose2d[] getModulePoses() {
//     Pose2d[] modulePoses = new Pose2d[modules.size()];
//     for (int i = 0; i < modules.size(); i++) {
//         var module = modules.get(i);
//         modulePoses[i] =
//                 getPose().transformBy(
//                     new Transform2d(
//                         module.().centerOffset, module.getAbsoluteHeading()));
//     }
//     return modulePoses;
// }


  /**
   * resets the odometry to the specified pose
   */
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(
      Rotation2d.fromRadians(gyro.getYaw()),
      new SwerveModulePosition[]{
        frontLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        rearLeft.getSwerveModulePosition(),
        rearRight.getSwerveModulePosition()}, pose);
  }

  /**
   * In case we'll ever need it
   * @return new yaw angle in radians (ideally)
   */
  public double setYawOffset() {
    gyro.setAngleAdjustment(-Math.PI / 2); // need to double check!
    return gyro.getYaw();
  }

  /**
   * x formation with wheels to prevent movement
   */
  public void setX(){
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
  }

  /**
   * sets the swerve ModuleStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, Drivetrain.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Zero the gyro heading
   */
  public void zeroHeading(){
    gyro.reset();
  }

  /* SYSID CMDS */
  public Command driveQuasistatic(SysIdRoutine.Direction direction){
    return driveRoutine.quasistatic(direction);
  }
  public Command driveDynamic(SysIdRoutine.Direction direction){
    return driveRoutine.dynamic(direction);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if gyro is inverted, getRotation2d() --- getAngle() can be negated
    odometry.update(
      gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(), rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()
    });
    poseEstimator.update(gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(), rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()});
    SmartDashboard.updateValues();
  }

  
}
