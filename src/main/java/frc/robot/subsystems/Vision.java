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

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.*;
import monologue.Logged;
import monologue.Annotations.Log;
public class Vision extends SubsystemBase implements Logged {
  private final PhotonCamera frontLeftCam; //, frontRightCam, rearLeftCam, rearRightCam;
  private PhotonPoseEstimator frontLeftEstimator;//, frontRightEstimator, rearLeftEstimator, rearRightEstimator;
  private AprilTagFieldLayout fieldLayout;

  private Matrix<N3, N1> currentStdDevs;

  Optional<EstimatedRobotPose> frontLeftUpdate; //, frontRightUpdate, rearLeftUpdate, rearRightUpdate;

  public Vision() {
    frontLeftCam = new PhotonCamera("2265-ironfish");
    // frontRightCam = new PhotonCamera("RightFront");
    // rearLeftCam = new PhotonCamera("LeftRear");
    // rearRightCam = new PhotonCamera("RightRear");

    frontLeftUpdate = Optional.empty();
    // frontRightUpdate = Optional.empty();
    // rearLeftUpdate = Optional.empty();
    // rearRightUpdate = Optional.empty();

    frontLeftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
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
  public Pose2d getCurrentPose(){
    var result = frontLeftCam.getLatestResult();
    boolean check = result.hasTargets();
    Pose2d botPose = new Pose2d();
    if(check) {
      var update = frontLeftEstimator.update(result);
      Pose3d currentPose3d = update.get().estimatedPose;
      botPose = currentPose3d.toPose2d();
    }
    return botPose;
  }

  public void printYaw(){
    var result = frontLeftCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    
    if(hasTargets){
    PhotonTrackedTarget target = result.getBestTarget();
    double yaw = target.getYaw();
    System.out.println("Yaw: " + yaw);
    double distanceToTarget = PhotonUtils.getDistanceToPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Pose2d(0.5, 0.5, new Rotation2d(0)));

    Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
  distanceToTarget, Rotation2d.fromDegrees(-target.getYaw()));
    }
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

  public PhotonTrackedTarget getTag(){
    PhotonTrackedTarget target = null;
    var result = frontLeftCam.getLatestResult();
    if(result.hasTargets()) {
     target = result.getBestTarget();
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

  @Log.NT
  public Pose3d get3d(){
    return getPose3d(getTag());
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
    printYaw();
    SmartDashboard.putData("3d pose", (Sendable)getPose3d(getTag()));
    SmartDashboard.putData("current pose", (Sendable)getCurrentPose());
    }
}
