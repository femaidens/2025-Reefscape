package frc.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveConstants.DriveSimConstants;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import monologue.Annotations.Log; 
// import monologue.Logged;

public class DriveSim extends SubsystemBase {

  private int dev;
  //private SimDouble angle;
  private double angle1;
  private Field2d m_field;
  private SwerveDriveOdometry odometry;
  private AHRS gyro;
  private ChassisSpeeds speeds;
  private final ModuleSim frontLeft;
  private final ModuleSim frontRight;
  private final ModuleSim rearLeft;
  private final ModuleSim rearRight;
//   public PPHolonomicDriveController holonomicDriveController;
//   private PIDConstants translationPID;
//   private PIDConstants rotationPID;
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
//   private RobotConfig config;

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
    // translationPID = new PIDConstants(Translation.PID.P,Translation.PID.I, Translation.PID.D );
    // rotationPID = new PIDConstants(Turn.PID.P, Turn.PID.I, Turn.PID.D);
    // holonomicDriveController = new PPHolonomicDriveController(translationPID, rotationPID);
    //angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    
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

        
        // try{
        //   config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //   // Handle exception as needed
        //   e.printStackTrace();
        // }

    //       AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  }

  

  public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    return this.run(() -> {
      double xVel = xSpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
      double yVel = ySpeed.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
      double rotVel = rotSpeed.getAsDouble() * Drivetrain.MAX_ROT_SPEED * Drivetrain.SPEED_FACTOR;

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, new Rotation2d(angle1));
      //angle.set(angle.get() + 0.02 * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
      angle1 +=  0.02 * Units.radiansToDegrees(speeds.omegaRadiansPerSecond);
      SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);
      moduleStates[0].optimize(moduleStates[0].angle);
      moduleStates[1].optimize(moduleStates[1].angle);
      moduleStates[2].optimize(moduleStates[2].angle);
      moduleStates[3].optimize(moduleStates[3].angle);
      
      // System.out.println(angle1);
      // System.out.println(frontLeft.getTurnAngle());
  

      frontLeft.setDesiredState(moduleStates[1]);
      frontRight.setDesiredState(moduleStates[0]);
      rearLeft.setDesiredState(moduleStates[3]);
      rearRight.setDesiredState(moduleStates[2]);
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
    odometry.update(new Rotation2d(angle1 ),
        new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(),
            rearLeft.getSwerveModulePosition(), rearRight.getSwerveModulePosition()
        });
        // System.out.println(angle1 );

    m_field.setRobotPose(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), new Rotation2d(Units.degreesToRadians(angle1 )));
    SmartDashboard.putData("Field", m_field);
  }

  public Pose2d getPose(){
    return m_field.getRobotPose();
  }
  public void resetOdometry(Pose2d pose){
    odometry.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds(){
    return speeds;
  }
  
  
  public void setChassisSpeeds(ChassisSpeeds speed){
    angle1 +=  0.02 * Units.radiansToDegrees(speed.omegaRadiansPerSecond);
      SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speed);
      moduleStates[0].optimize(moduleStates[0].angle);
      moduleStates[1].optimize(moduleStates[1].angle);
      moduleStates[2].optimize(moduleStates[2].angle);
      moduleStates[3].optimize(moduleStates[3].angle);
      // System.out.println(angle1);
      // System.out.println(frontLeft.getTurnAngle());
      frontLeft.setDesiredState(moduleStates[0]);
      frontRight.setDesiredState(moduleStates[1]);
      rearLeft.setDesiredState(moduleStates[2]);
      rearRight.setDesiredState(moduleStates[3]);
  }

   
}