package frc.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveConstants.DriveSimConstants;
import frc.robot.subsystems.DriveConstants.Drivetrain;

public class DriveSim extends SubsystemBase {

  private int dev;
  private SimDouble angle;
  private Field2d m_field;
  private SwerveDriveOdometry odometry;
  private AHRS gyro;
  private final ModuleSim frontLeft;
  private final ModuleSim frontRight;
  private final ModuleSim rearLeft;
  private final ModuleSim rearRight;
  // private Pose2d poseA;
  // private Pose2d poseB;
  // private Pose3d poseA3d;
  // private Pose3d poseB3d;
  // private StructPublisher<Pose2d> publisherPose;
  // private StructArrayPublisher<Pose2d> arrayPublisher;
  // private StructPublisher<Pose3d> publisherSwerve;
  // private StructArrayPublisher<Pose3d> arrayPublisherSwerve;

  private StructArrayPublisher<SwerveModuleState> publisher;
  private StructArrayPublisher<SwerveModuleState> desiredPublisher;

  private List<ModuleSim> modules;
  // private Trajectory m_trajectory;

  public DriveSim() {

    frontLeft = new ModuleSim();
    frontRight = new ModuleSim();
    rearRight = new ModuleSim();
    rearLeft = new ModuleSim();

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);

    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("My States", SwerveModuleState.struct).publish();

    desiredPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();
    
    dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    gyro = new AHRS(NavXComType.kI2C);
    angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    m_field = new Field2d();
    odometry = new SwerveDriveOdometry(
        Drivetrain.kDriveKinematics,
        new Rotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition()
        });
    // m_trajectory = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(20,50,Rotation2d.fromDegrees(0)),
    // List.of(new Translation2d(1,1), new Translation2d(2,-1)),
    // new Pose2d(3,0,Rotation2d.fromDegrees(0)),
    // new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
    // );

    // poseA = new Pose2d();
    // poseB = new Pose2d(); //creates a 2d representation of the swerve drive
    // publisherPose = NetworkTableInstance.getDefault().getStructTopic("MyPose",
    // Pose2d.struct).publish();
    // arrayPublisher =
    // NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray",
    // Pose2d.struct).publish();
    // poseA3d = new Pose3d();
    // poseB3d = new Pose3d(); //creates a 3d representation of the swerve drive
    // publisherSwerve = NetworkTableInstance.getDefault().getStructTopic("MyPose",
    // Pose3d.struct).publish();
    // arrayPublisherSwerve =
    // NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray",
    // Pose3d.struct).publish();
    // arrayPublisherSwerve.set(new Pose3d[] {poseA3d, poseB3d});
    // arrayPublisher.set(new Pose2d[] {poseA, poseB});

    // m_field.setRobotPose(m_odometry.getPoseMeters());
    // m_field.setRobotPose(7.488986, 6.662293, Rotation2d.fromDegrees(.392));

  }

  public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    return this.run(() -> {
      double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
      double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
      double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, new Rotation2d(angle.get()));
      angle.set(angle.get() + 0.02 * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
      SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);
      
      System.out.println(frontLeft.getTurnAngle());
      frontLeft.setDesiredState(moduleStates[0]);
      frontRight.setDesiredState(moduleStates[1]);
      rearLeft.setDesiredState(moduleStates[2]);
      rearRight.setDesiredState(moduleStates[3]);
    });

  }

  public Command driveForward() {
    return this.run(() -> {
      frontLeft.setDriveVoltage(DriveSimConstants.DRIVE_FORWARD_VOLTAGE);
      frontRight.setDriveVoltage(DriveSimConstants.DRIVE_FORWARD_VOLTAGE);
      rearLeft.setDriveVoltage(DriveSimConstants.DRIVE_FORWARD_VOLTAGE);
      rearRight.setDriveVoltage(DriveSimConstants.DRIVE_FORWARD_VOLTAGE);
    });
  }

  public void close() {
    frontLeft.setDriveVoltage(0);
    frontRight.setDriveVoltage(0);
    rearLeft.setDriveVoltage(0);
    rearRight.setDriveVoltage(0);
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    };
  }

  public SwerveModuleState[] getDesiredStates(){
    return new SwerveModuleState[] {
      frontLeft.desiredState,
      frontRight.desiredState,
      rearLeft.desiredState,
      rearRight.desiredState
    };
  }

  @Override
  public void simulationPeriodic() {
    // m_field.setRobotPose(1, 6, Rotation2d.fromDegrees(100));
    // publisherPose.set(poseA);
    // publisherSwerve.set(poseA3d);
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    // m_field.getObject("traj").setTrajectory(m_trajectory);
    publisher.set(this.getStates());
    desiredPublisher.set(this.getDesiredStates());
    odometry.update(new Rotation2d(Units.degreesToRadians(gyro.getYaw())),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()
        });

    m_field.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), new Rotation2d(Units.degreesToRadians(angle.get())));
    SmartDashboard.putData("Field", m_field);

  }

}
