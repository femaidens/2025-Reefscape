// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveConstants.Drivetrain;



public class Vision {
  private final PhotonCamera frontLeftCam, frontRightCam, rearLeftCam, rearRightCam;
  private PhotonPoseEstimator frontLeftEstimator,frontRightEstimator, rearLeftEstimator, rearRightEstimator;
  private AprilTagFieldLayout fieldLayout;

  private Matrix<N3, N1> currentStdDevs;

  Optional<EstimatedRobotPose> frontLeftUpdate, frontRightUpdate, rearLeftUpdate, rearRightUpdate;


  public Vision() {
    frontLeftCam = new PhotonCamera("LeftFront");
    frontRightCam = new PhotonCamera("RightFront");
    rearLeftCam = new PhotonCamera("LeftRear");
    rearRightCam = new PhotonCamera("RightRear");

    frontLeftUpdate = Optional.empty();
    frontRightUpdate = Optional.empty();
    rearLeftUpdate = Optional.empty();
    rearRightUpdate = Optional.empty();

    frontLeftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kFrontLeftCamToCenter);
    frontRightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kFrontRightCamToCenter);
    rearLeftEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRearLeftCamToCenter);
    rearRightEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRearRightCamToCenter);

    frontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    frontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rearLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    rearRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    frontLeftEstimator.setFieldTags(fieldLayout);
    frontRightEstimator.setFieldTags(fieldLayout);
    rearLeftEstimator.setFieldTags(fieldLayout);
    rearRightEstimator.setFieldTags(fieldLayout);
  }
  
  private Optional<EstimatedRobotPose> updateEstimatedGlobalPoses() {
    if (frontLeftEstimator == null) {
      // Configuration failed, there's nothing we can do
      return Optional.empty();
    }
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : frontLeftCam.getAllUnreadResults()) {
      visionEst = frontLeftEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;

    // for (int i = 0; i < estimators.length; i++) {
    //   var unread = cameras[i].getAllUnreadResults();
    //   PhotonPipelineResult result;
    //   if (unread.size() > 1) {
    //     // gets the latest result if there are multiple unread results
    //     int maxIndex = 0;
    //     double max = 0;
    //     int unreadLength = unread.size();
    //     for (int ie = 0; ie < unreadLength; ie++) {
    //       double temp = unread.get(ie).getTimestampSeconds();
    //       if (temp > max) {
    //         max = temp;
    //         maxIndex = ie;
    //       }
    //     }
    //     result = unread.get(maxIndex);
    //     lastResults[i] = result;
    //   } else if (unread.size() == 1) {
    //     result = unread.get(0);
    //     lastResults[i] = result;
    //   } else {
    //     result = lastResults[i];
    //   }
    //   var estimate = estimators[i].update(result);
    // }
  }


  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      currentStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

     // Precalculation - see how many tags we found, and calculate an average-distance metric
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
        // No tags visible. Default to single-tag std devs
        currentStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        currentStdDevs = estStdDevs;
      }
    }
  }
  

  public Matrix<N3, N1> getEstimationStdDevs() {
    return currentStdDevs;
  }
}
