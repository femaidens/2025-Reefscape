// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.SignalLogger;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import monologue.Annotations.Log;
import monologue.Logged;

public class Drive extends SubsystemBase implements Logged {

  private final ModuleKraken frontLeft;
  private final ModuleKraken frontRight;
  private final ModuleKraken rearLeft;
  private final ModuleKraken rearRight;

  private final List<ModuleKraken> modules;

  private final AHRS gyro;

  public final SwerveDriveOdometry odometry;

  private final SysIdRoutine driveRoutine;

  private ChassisSpeeds speeds = new ChassisSpeeds();

  /** Creates a new Drive. */
  public Drive() {
    // frontLeft = new ModuleSpark(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN, Translation.FRONT_LEFT_ANGOFFSET);
    // frontRight = new ModuleSpark(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN, Translation.FRONT_RIGHT_ANGOFFSET);
    // rearLeft = new ModuleSpark(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN, Translation.REAR_LEFT_ANGOFFSET);
    // rearRight = new ModuleSpark(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN, Translation.REAR_RIGHT_ANGOFFSET);
    frontLeft = new ModuleKraken(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN, DrivetrainPorts.FRONT_LEFT_CANCODER, Translation.FRONT_LEFT_MAG_OFFSET, Translation.FRONT_LEFT_ANGOFFSET, false);
    frontRight = new ModuleKraken(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN, DrivetrainPorts.FRONT_RIGHT_CANCODER, Translation.FRONT_RIGHT_MAG_OFFSET, Translation.FRONT_RIGHT_ANGOFFSET, false);
    rearLeft = new ModuleKraken(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN, DrivetrainPorts.REAR_LEFT_CANCODER, Translation.REAR_LEFT_MAG_OFFSET, Translation.REAR_LEFT_ANGOFFSET, false);
    rearRight = new ModuleKraken(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN, DrivetrainPorts.REAR_RIGHT_CANCODER, Translation.REAR_RIGHT_MAG_OFFSET, Translation.REAR_RIGHT_ANGOFFSET, false);

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

    // totally not sure, would need to check
    
    gyro = new AHRS(NavXComType.kMXP_SPI);

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

      driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null,
        //  Volts.of(2).per(Seconds.of(1)),
        //  Volts.of(9),
         null,
         (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
          volts -> modules.forEach(m -> m.setDriveVoltage(volts.in(Units.Volts))),
          null,
          this));
  

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
  public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed){
    double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    // return this.run(
    //   () -> {
    //     for(int i = 0; i < modules.size(); i++){
    //       modules.get(i).setDesiredState(moduleStates[i]);
    //     }
    //   }
    // );
    setModuleStates(moduleStates);
  }

  public void driveRobotRelative(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed){
    double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

    speeds = new ChassisSpeeds(xVel, yVel, rotVel);
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    // return this.run(
    //   () -> {
    //     for(int i = 0; i < modules.size(); i++){
    //       modules.get(i).setDesiredState(moduleStates[i]);
    //     }
    //   }
    // );
    setModuleStates(moduleStates);
  }
  
  /**
   * sets the swerve ModuleStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, Drivetrain.MAX_SPEED);
    frontLeft.setDesiredStateNoPID(desiredStates[0]); //frontLeft.setDesiredStateNoPID(desiredStates[1]);
    frontRight.setDesiredStateNoPID(desiredStates[1]); //frontRight.setDesiredStateNoPID(desiredStates[0]);
    rearLeft.setDesiredStateNoPID(desiredStates[2]); //rearLeft.setDesiredStateNoPID(desiredStates[3]);
    rearRight.setDesiredStateNoPID(desiredStates[3]); //rearRight.setDesiredStateNoPID(desiredStates[2]);
  }

  public void setChassisSpeeds(ChassisSpeeds  speedd){
    // ChassisSpeeds x = ChassisSpeeds.fromFieldRelativeSpeeds(speedd, getAngle());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speedd);
    setModuleStates(moduleStates);
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return speeds;
  }

  public ChassisSpeeds getCurrentChassisSpeeds(){
    ChassisSpeeds spede= DriveConstants.Drivetrain.kDriveKinematics.toChassisSpeeds(
      getSwerveModuleStates()[0],  getSwerveModuleStates()[1], getSwerveModuleStates()[2], getSwerveModuleStates()[3] 
    );
    return spede;
  }

  public SwerveModulePosition[] getSwerveModulePosition(){
     SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(), rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()};
     return swerveModulePositions;
  }

  /**
   * x formation with wheels to prevent movement
   */
  public Command setXCmd() {
    return this.run(
      () -> {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI/4)));
      }
    );
  }


  /**
   * Sets wheels straight for sysid
   */
  public Command setStraightCmd(){
    return this.run(
      () -> {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        //only rear right is acting up, consider changing it to 180 degrees
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      }
    );
  }

  public Command driveStraightCmd(){
    return this.run(
      () -> {
        frontLeft.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
        frontRight.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
        rearLeft.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
        rearRight.setDesiredState(new SwerveModuleState(.2, Rotation2d.fromRadians(0)));
      }
    );
  }

  /**
   * @return currently-estimated pose of robot
   */
  @Log.NT
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  @Log.NT
  public SwerveModuleState[] getSwerveModuleStates(){
    return modules.stream().map(m -> m.getState()).toArray(SwerveModuleState[]::new);
  }

  @Log.NT
  public SwerveModuleState[] getDesiredSwerveModuleStates(){
    return modules.stream().map(m -> m.getDesiredState()).toArray(SwerveModuleState[] :: new);
  }

  @Log.NT
  public double[] getVoltage(){
    double[] voltages = {frontLeft.getVoltage(), frontRight.getVoltage(), rearLeft.getVoltage(), rearRight.getVoltage()};
    return voltages;
  }

  @Log.NT
  public double[] getAbsolutes(){
    double[] absolutes = {frontLeft.getAbsolute(), frontRight.getAbsolute(), rearLeft.getAbsolute(), rearRight.getAbsolute()};
    return absolutes;
  }

  /**
   * resets the odometry to the specified pose
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      Rotation2d.fromRadians(gyro.getYaw()),
      new SwerveModulePosition[]{
        frontLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        rearLeft.getSwerveModulePosition(),
        rearRight.getSwerveModulePosition()}, pose);
  }

   /**
   * Gets the angle of the gyro in radians (ideally)
   * @return in radians
   */
  @Log.NT
  public double getAngle(){
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

  public Command resetGyro(){
    return this.runOnce(
      () -> zeroHeading()
    );
  }

  /* SYSID CMDS */
  public Command driveQuasistatic(SysIdRoutine.Direction direction){
    System.out.println("RUNNING");
    System.out.println("RUNNING");
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
        frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(), rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()
    });
    //SmartDashboard.getNumber("Angle", getAngle());
    SmartDashboard.putNumber("Gyro Angle", getAngle());
    SmartDashboard.updateValues();
    SmartDashboard.putNumber("angle", getAngle());
  }
}