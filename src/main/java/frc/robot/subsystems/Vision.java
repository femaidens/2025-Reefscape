// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
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
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveConstants.Drivetrain;



public class Vision {
  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] estimators;
  private final PhotonPipelineResult[] lastResults;
  private final PoseEstimator poseEstimator;
  private Matrix<N3, N1> currentStdDevs;

  public Vision() {
    cameras = new PhotonCamera[4];
    estimators = new PhotonPoseEstimator[4];
    lastResults = new PhotonPipelineResult[4];
    poseEstimator = new PoseEstimator<>(null, null, currentStdDevs, currentStdDevs)
    for(int i = 0; i < cameras.length; i++) {
      cameras[i] = new PhotonCamera("cam" + i);
      PhotonPoseEstimator estimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam);
      estimators[i] = estimator;
      lastResults[i] = new PhotonPipelineResult();
    }
  }
  
  private void updateEstimatedGlobalPoses() {

    for (int i = 0; i < estimators.length; i++) {
      var unread = cameras[i].getAllUnreadResults();
      PhotonPipelineResult result;
      if (unread.size() > 1) {
        // gets the latest result if there are multiple unread results
        int maxIndex = 0;
        double max = 0;
        int unreadLength = unread.size();
        for (int ie = 0; ie < unreadLength; ie++) {
          double temp = unread.get(ie).getTimestampSeconds();
          if (temp > max) {
            max = temp;
            maxIndex = ie;
          }
        }
        result = unread.get(maxIndex);
        lastResults[i] = result;
      } else if (unread.size() == 1) {
        result = unread.get(0);
        lastResults[i] = result;
      } else {
        result = lastResults[i];
      }
      var estimate = estimators[i].update(result);
    }
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
