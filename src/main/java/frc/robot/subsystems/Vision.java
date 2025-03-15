// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import frc.robot.Constants;
import frc.robot.Constants.*;
import monologue.Logged;
import monologue.Annotations.Log;
public class Vision extends SubsystemBase implements Logged {
  private final PhotonCamera frontLeftCam, frontRightCam; //rearLeftCam, rearRightCam;
  private PhotonPoseEstimator frontLeftEstimator;//, frontRightEstimator, rearLeftEstimator, rearRightEstimator;
  private AprilTagFieldLayout fieldLayout;

  private Matrix<N3, N1> currentStdDevs;

  @Log.NT
  double forward;

  @Log.NT
  double turn;

  @Log.NT
  double targetRange;

  @Log.NT
  double targetXe;

  @Log.NT
  double tilt = 0;


  Optional<EstimatedRobotPose> frontLeftUpdate; //, frontRightUpdate, rearLeftUpdate, rearRightUpdate;
  private Drive drive;

  public Vision() {
    turn = 0.0;
    forward = 0.0;
    targetRange = 0.0;
    frontLeftCam = new PhotonCamera("2265-greenfish");
    drive = new Drive();
    frontRightCam = new PhotonCamera("2265-ironfish");
    // rearLeftCam = new PhotonCamera("LeftRear");
    // rearRightCam = new PhotonCamera("RightRear");

    frontLeftUpdate = Optional.empty();
    // frontRightUpdate = Optional.empty();
    // rearLeftUpdate = Optional.empty();
    // rearRightUpdate = Optional.empty();

    frontLeftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
        VisionConstants.kFrontLeftCamToCenter);
    // frontRightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     VisionConstants.kFrontRightCamToCenter);
    // rearLeftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     VisionConstants.kRearLeftCamToCenter);
    // rearRightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     VisionConstants.kRearRightCamToCenter);

    frontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // frontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // rearLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // rearRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    frontLeftEstimator.setFieldTags(fieldLayout);
    // frontRightEstimator.setFieldTags(fieldLayout);
    // rearLeftEstimator.setFieldTags(fieldLayout);
    // rearRightEstimator.setFieldTags(fieldLayout);
  }
  @Log.NT
  public Pose2d getCurrentPose(){
    var result = frontLeftCam.getAllUnreadResults();
    boolean check = false;
    Pose2d botPose = new Pose2d();
    if(result.size() > 0){
      // check = result.get(0).hasTargets();
      var update = frontLeftEstimator.update(result.get(0));
      Pose3d currentPose3d = update.get().estimatedPose;
      botPose = currentPose3d.toPose2d();
    }

    
    // if(check) {
      
    // }
    return botPose;
  }

  public Command printYaw(){
    return this.run(() -> {
      var result = frontLeftCam.getAllUnreadResults();
    boolean hasTargets = result.get(0).hasTargets();
    
    if(hasTargets){
    PhotonTrackedTarget target = result.get(0).getBestTarget();
    double yaw = target.getYaw(); 

    if(yaw > 0){
      drive.drive(()-> 0.0, ()-> 0.0, ()-> 0.1);
    }else if(yaw < 0){
      drive.drive(()-> 0.0, ()->0.0, ()-> -0.1);
    }else{
      drive.drive(()-> 0.0, ()-> 0.0, ()-> 0.0);
    }
    System.out.println("Yaw: " + yaw);
  
  }});
}
  public void driveFromVision(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed){
    drive.drive(xSpeed, ySpeed, rotSpeed);
  }

  public Command resetGyroFromVision(){
    return drive.resetGyro();
  }
  public Command driveTranslational(){
    return this.run(()->{
      var result = frontLeftCam.getAllUnreadResults();
    boolean hasTargets = result.get(0).hasTargets();
    
    if(hasTargets){
    PhotonTrackedTarget target = result.get(0).getBestTarget();
    if(this.distanceToTarget(target) > 0.5){
      drive.drive(()-> 0.1, ()-> 0.0, ()-> 0.0);
    }else if(this.distanceToTarget(target) < 0.5){
      drive.drive(()-> -0.1, ()->0.0, ()-> 0.0);
    }else{
      drive.drive(()-> 0.0, ()-> 0.0, ()-> 0.0);
    }
    }
    });
  }
  public Command stopDriving(){
    return this.runOnce(()-> drive.drive(()-> 0.0, () -> 0.0, () -> 0.0));
  }
  //   double distanceToTarget = PhotonUtils.getDistanceToPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Pose2d(0.5, 0.5, new Rotation2d(0)));

  //   Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
  // distanceToTarget, Rotation2d.fromDegrees(-target.getYaw()));
  public void funky(){
    double range = .165;
    boolean targetVisible = false;
    double[] speeds = {0, 0}; //forward, turn
        double targetYaw = 0.0;
        var results = frontLeftCam.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 6) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.25, // Measured with a tape measure, or in CAD.
                                        .2, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-20.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;

                        if (targetVisible) {
                          // Driver wants auto-alignment to tag 7
                          // And, tag 7 is in sight, so we can turn toward it.
                          // Override the driver's turn and fwd/rev command with an automatic one
                          // That turns toward the tag, and gets the range right.
                          speeds[1] =
                                  (targetYaw) * DriveConstants.Translation.PID.P * DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
                          speeds[0] =
                                  -(range - targetRange) * VisionConstants.PID.P * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
                          forward = speeds[0];
                          turn = speeds[1];
                      }
                    }
                }
            }
        }
        System.out.println(targetRange + "   " + targetYaw);
        drive.drive(() -> speeds[0], () -> 0, () -> speeds[1]);
      // });
  }

  /**
   * Uses the target corners of the photontrackedtarget, yaw, and pitch to align (without pose estimation)
   * @return
   */
  public Command funkierLeft(){
    return this.run(() -> {
      var results = frontRightCam.getAllUnreadResults();
      // boolean hasTargets = result.get(0).hasTargets();
      // double xSpeed = 0; //forward
      // double ySpeed = 0; //strafe
      // double thetaSpeed = 0;
      double[] speeds = {0, 0, 0}; // order: forward (xspeed), strafe (yspeed), theta (thetaspeed)
      if(results.size() > 0 && results.get(0).hasTargets()){
        var result = results.get(results.size() - 1);
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 6) {
            // PhotonTrackedTarget target = result.get(0).getBestTarget();
            List<TargetCorner> targetCorners = target.getDetectedCorners();
            // corners as specified by getDetectedCorners()
            TargetCorner bottomLeft = targetCorners.get(0);
            TargetCorner bottomRight = targetCorners.get(1);
            TargetCorner topRight = targetCorners.get(2);
            TargetCorner topLeft = targetCorners.get(3);
        
            double targetArea = target.area; // area in percentage?
            if(targetArea > VisionConstants.GOAL_AREA){
              speeds[0] = -(targetArea-VisionConstants.GOAL_AREA) * 0.01 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            } else if(targetArea < VisionConstants.GOAL_AREA){
              speeds[0] = (targetArea-VisionConstants.GOAL_AREA) * 0.01 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            }

            double targetX = (bottomLeft.x + bottomRight.x + topLeft.x + topRight.x) / 4; // avg to find mid point of apriltag, should be like x position of crosshair\
            targetXe = targetX;
            if(Math.abs(targetX - VisionConstants.GOAL_X) > 20){ // current tag is farther right than desired
              speeds[1] = (targetX-VisionConstants.GOAL_X) * 0.001 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            // }else if(targetX < VisionConstants.GOAL_X){
            //   speeds[1] = -(targetX-VisionConstants.GOAL_X) * 0.001 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            }
            
            tilt = (bottomLeft.y - topLeft.y) / (bottomRight.y - topRight.y); // just to compare lengths of left & right side of fidicial id to determine which way its angled
            if(tilt > 1.02) { // left side of id is longer than right
              speeds[2] = -(tilt-1) * VisionConstants.PID.P * DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
            } else if(tilt < 0.97) { // left side is shorter than right
              speeds[2] = (tilt - 1) * VisionConstants.PID.P * DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
            } else {
              speeds[2] = 0;
            }
            System.out.println("Tag location: " + targetX);
      }
    }}
      drive.drive(() -> speeds[0], () -> speeds[1], () -> speeds[2]); //lowkey crazy workaround for local variables issue with lambda
    });
  }
/**
 * just to confirm what getDetectedCorners actually gives, seems to give positions 
 * relative to top left of camera stream
 */
  public void funkierPrint(){
    // var result = frontLeftCam.getAllUnreadResults();
    // if(result.size() > 0 && result.get(0).hasTargets()){
    //   PhotonTrackedTarget target = result.get(0).getBestTarget();
    //   List<TargetCorner> corners = target.getDetectedCorners();
    //   System.out.println(Arrays.toString(corners.toArray()));
    // }
    System.out.println(targetXe);
  }

  public Optional<EstimatedRobotPose> updateEstimatedGlobalPoses() {
    if (frontLeftEstimator == null) {
      return Optional.empty();
    }
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : frontLeftCam.getAllUnreadResults()) {
      visionEst = frontLeftEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }
  public double distanceToTarget(PhotonTrackedTarget target){
    double distance =
      PhotonUtils.calculateDistanceToTargetMeters(
              0, 
              0, 
              Units.degreesToRadians(0), 
              Units.degreesToRadians(target.getPitch()));
    return distance;
  }

  public Command funkierRight(){
    return this.run(() -> {
      var results = frontLeftCam.getAllUnreadResults();
      // boolean hasTargets = result.get(0).hasTargets();
      // double xSpeed = 0; //forward
      // double ySpeed = 0; //strafe
      // double thetaSpeed = 0;
      double[] speeds = {0, 0, 0}; // order: forward (xspeed), strafe (yspeed), theta (thetaspeed)
      if(results.size() > 0 && results.get(0).hasTargets()){
        var result = results.get(results.size() - 1);
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 6) {
            // PhotonTrackedTarget target = result.get(0).getBestTarget();
            List<TargetCorner> targetCorners = target.getDetectedCorners();
            // corners as specified by getDetectedCorners()
            TargetCorner bottomLeft = targetCorners.get(0);
            TargetCorner bottomRight = targetCorners.get(1);
            TargetCorner topRight = targetCorners.get(2);
            TargetCorner topLeft = targetCorners.get(3);
        
            double targetArea = target.area; // area in percentage?
            if(targetArea > VisionConstants.GOAL_AREA){
              speeds[0] = -(targetArea-VisionConstants.GOAL_AREA) * 0.01 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            } else if(targetArea < VisionConstants.GOAL_AREA){
              speeds[0] = (targetArea-VisionConstants.GOAL_AREA) * 0.01 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            }

            double targetX = (bottomLeft.x + bottomRight.x + topLeft.x + topRight.x) / 4; // avg to find mid point of apriltag, should be like x position of crosshair\
            targetXe = targetX;
            if(Math.abs(targetX - VisionConstants.GOAL_X) > 20){ // current tag is farther right than desired
              speeds[1] = (targetX-VisionConstants.GOAL_X) * 0.001 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            // }else if(targetX < VisionConstants.GOAL_X){
            //   speeds[1] = -(targetX-VisionConstants.GOAL_X) * 0.001 * DriveConstants.Translation.MAX_TRANSLATION_VELOCITY.in(MetersPerSecond);
            }
            
            tilt = (bottomLeft.y - topLeft.y) / (bottomRight.y - topRight.y); // just to compare lengths of left & right side of fidicial id to determine which way its angled
            if(tilt > 1.02) { // left side of id is longer than right
              speeds[2] = -(tilt-1) * VisionConstants.PID.P * DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
            } else if(tilt < 0.97) { // left side is shorter than right
              speeds[2] = (tilt - 1) * VisionConstants.PID.P * DriveConstants.Turn.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
            } else {
              speeds[2] = 0;
            }
            System.out.println("Tag location: " + targetX);
      }
    }}
      drive.drive(() -> speeds[0], () -> speeds[1], () -> speeds[2]); //lowkey crazy workaround for local variables issue with lambda
    });
  }

  public PhotonTrackedTarget getTag(){
    PhotonTrackedTarget target = null;
    var result = frontRightCam.getAllUnreadResults();
    if(result.get(0).hasTargets()) {
     target = result.get(0).getBestTarget();
    }
    return target;
  }


  public void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      currentStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      //use all seen tags to calculate an average distance metric
      for (var tgt : targets) {
        var tagPose = VisionConstants.kTagLayout.getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist += tagPose.get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        currentStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        avgDist /= numTags;
        if (numTags > 1)
          estStdDevs = VisionConstants.kMultiTagStdDevs;
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        currentStdDevs = estStdDevs;
      }
    }
  }
  
  public Pose3d getPose3d(PhotonTrackedTarget target){
    // if (Constants.VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).get(), VisionConstants.kFrontLeftCamToCenter);
     return robotPose;
    // }
    // return robotPose;
  }


  public Pose2d getPose2d(PhotonTrackedTarget target){
    
    Pose2d robotPose = getPose3d(target).toPose2d();
    return robotPose;
  }


  public Rotation2d getYawDistance(PhotonTrackedTarget target, Pose2d targetPose){
     Rotation2d targetYaw = PhotonUtils.getYawToPose(getPose2d(target), targetPose);
     return targetYaw;
  }


  public Matrix<N3, N1> getEstimationStdDevs() {
    return currentStdDevs;
  }

  // public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
  //   frontLeftEstimator.addVisionMeaurement(visionMeasurement, timestampSeconds);
  // }

  // public boolean isFacing(Pose2d pose, Pose2d curPose2d) {    

  //   // Disregard measurements too far away from odometry
  //   // this can be tuned to find a threshold that helps us remove jumping vision
  //   // poses
  //   return (Math.abs(pose.getX() - curPose2d.getX()) <= 0.1)
  //       && (Math.abs(pose.getY() - curPose2d.getY()) <= 0.1);
  // }
  @Override
  public void periodic(){
    // printYaw();
    // SmartDashboard.putData("3d pose", (Sendable)getPose3d(getTag()));
    // SmartDashboard.putData("current pose", (Sendable)getCurrentPose());
    funkierPrint();
  }
}
