package frc.robot.subsystems;

import java.util.List;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;

public class DriveSim extends SubsystemBase{
    
    private  int dev; 
    private  SimDouble angle; 
    // private  Field2d m_field; 
    private  Odometry m_odometry; 
    private  AHRS gyro; 
    private final ModuleSim frontLeft;
    private final ModuleSim frontRight;
    private final ModuleSim rearLeft;
    private final ModuleSim rearRight;
    private Pose2d poseA;
    private Pose2d poseB;
    private Pose3d poseA3d;
    private Pose3d poseB3d;
    private StructPublisher<Pose2d> publisherPose;
    private StructArrayPublisher<Pose2d> arrayPublisher;
    private StructPublisher<Pose3d> publisherSwerve;
    private StructArrayPublisher<Pose3d> arrayPublisherSwerve;
    // private Trajectory m_trajectory; 
  
    public DriveSim(){
      // frontLeft = new ModuleSpark(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN, Translation.FRONT_LEFT_ANGOFFSET);
      // frontRight = new ModuleSpark(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN, Translation.FRONT_RIGHT_ANGOFFSET);
      // rearLeft = new ModuleSpark(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN, Translation.REAR_LEFT_ANGOFFSET);
      // rearRight = new ModuleSpark(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN, Translation.REAR_RIGHT_ANGOFFSET);

      frontLeft = new ModuleSim();
      frontRight = new ModuleSim();
      rearRight = new ModuleSim();
      rearLeft = new ModuleSim();
        // dev = SimDeviceDataJNI.getSimDeviceHandle("name"); 
        // gyro = new AHRS(NavXComType.kI2C); 
        // angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")); 
        // angle.set(5.0); 
        // m_field = new Field2d();
        // m_odometry = new SwerveDriveOdometry(
        //   Drivetrain.kDriveKinematics, 
        //   gyro.getRotation2d(), 
        //   new SwerveModulePosition[] {
        //     frontLeft.getSwerveModulePosition(),
        //     frontRight.getSwerveModulePosition(),
        //     rearLeft.getSwerveModulePosition(),
        //     rearRight.getSwerveModulePosition()
        //   });
          //  m_trajectory = TrajectoryGenerator.generateTrajectory(
          //               new Pose2d(20,50,Rotation2d.fromDegrees(0)),
          //               List.of(new Translation2d(1,1), new Translation2d(2,-1)), 
          //               new Pose2d(3,0,Rotation2d.fromDegrees(0)),
          //               new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
          //  ); 

          poseA = new Pose2d();
          // poseB = new Pose2d(); //creates a 2d representation of the swerve drive
          // publisherPose = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
          // arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
          // poseA3d = new Pose3d();
          // poseB3d = new Pose3d(); //creates a 3d representation of the swerve drive
          // publisherSwerve = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
         // arrayPublisherSwerve = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();      
          //arrayPublisherSwerve.set(new Pose3d[] {poseA3d, poseB3d});
         // arrayPublisher.set(new Pose2d[] {poseA, poseB});
          

        // SmartDashboard.putNumber("angle", angle.get());  
        // SmartDashboard.putData("Field", m_field); 

        // this is my only change so i can push 
    // }
    // @Override
    // public void periodic(){

    }
    public void driveForward(){
      frontLeft.setDriveVoltage(12);
      frontRight.setDriveVoltage(12);
      rearLeft.setDriveVoltage(12);
      rearRight.setDriveVoltage(12);
    }
    public void close(){
      frontLeft.setDriveVoltage(0);
      frontRight.setDriveVoltage(0);
      rearLeft.setDriveVoltage(0);
      rearRight.setDriveVoltage(0);
    }

  @Override
  public void simulationPeriodic() {
    // m_field.setRobotPose(1, 6, Rotation2d.fromDegrees(100));
    // publisherPose.set(poseA);
    // publisherSwerve.set(poseA3d);
    // m_field.setRobotPose(m_odometry.getPoseMeters()); 
    // m_field.getObject("traj").setTrajectory(m_trajectory);

   }

}
