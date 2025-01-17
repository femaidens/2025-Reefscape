package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    
    private final int dev; 
    private final SimDouble angle; 
    private final Field2d m_field; 
    private final Odometry m_odometry; 
    private final AHRS gyro; 
    private final ModuleSpark frontLeft;
    private final ModuleSpark frontRight;
    private final ModuleSpark rearLeft;
    private final ModuleSpark rearRight;
   private Pose2d poseA;
    private Pose2d poseB;
    private Pose3d poseA3d;
    private Pose3d poseB3d;
    private StructPublisher<Pose2d> publisherPose;
    private StructArrayPublisher<Pose2d> arrayPublisher;
    private StructPublisher<Pose3d> publisherSwerve;
    private StructArrayPublisher<Pose3d> arrayPublisherSwerve;
  
    public DriveSim(){
      frontLeft = new ModuleSpark(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN, Translation.FRONT_LEFT_ANGOFFSET);
      frontRight = new ModuleSpark(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN, Translation.FRONT_RIGHT_ANGOFFSET);
      rearLeft = new ModuleSpark(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN, Translation.REAR_LEFT_ANGOFFSET);
      rearRight = new ModuleSpark(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN, Translation.REAR_RIGHT_ANGOFFSET);
        dev = SimDeviceDataJNI.getSimDeviceHandle("name"); 
        gyro = new AHRS(NavXComType.kI2C); 
        angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")); 
        angle.set(5.0); 
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field); 
        m_odometry = new SwerveDriveOdometry(
          Drivetrain.kDriveKinematics, 
          gyro.getRotation2d(), 
          new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(),
            rearRight.getSwerveModulePosition()
          });
           

          poseA = new Pose2d();
          poseB = new Pose2d(); //creates a 2d representation of the swerve drive
          publisherPose = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
          arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
          poseA3d = new Pose3d();
          poseB3d = new Pose3d(); //creates a 3d representation of the swerve drive
          publisherSwerve = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
          arrayPublisherSwerve = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();      
          arrayPublisherSwerve.set(new Pose3d[] {poseA3d, poseB3d});
          arrayPublisher.set(new Pose2d[] {poseA, poseB});
          m_field.setRobotPose(m_odometry.getPoseMeters()); 

        SmartDashboard.putNumber("angle", angle.get());  
        
        // this is my only change so i can push 
    }
    @Override
    public void periodic(){

    }
  @Override
  public void simulationPeriodic() {
    publisherPose.set(poseA);
    publisherSwerve.set(poseA3d);
        
  }

}
