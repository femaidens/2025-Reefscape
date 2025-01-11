// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.Optional;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {
    final PhotonPoseEstimator poseEstimators; 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Optional<Pose2d>[] getEstimates(PhotonPipelineResult[] results,
      PhotonPoseEstimator[] photonEstimator) {
    ArrayList<Optional<Pose2d>> estimates = new ArrayList<>();
    for (int i = 0; i < results.length; i++) {
      PhotonPipelineResult result = results[i];
      if (result.hasTargets()) {
        var est = photonEstimator[i].update();
        if (est.isPresent() && goodResult(result)) {
          estimates.add(Optional.of(est.get().estimatedPose.toPose2d()));
        } else {
          estimates.add(Optional.empty());
        }
      } else {
        estimates.add(Optional.empty());
      }
    }
    Optional<Pose2d>[] estimatesArray = estimates.toArray(new Optional[0]);
    return estimatesArray;
  }

  public boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets();
  }
}
