// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;


/** Add your docs here. */
public class PhotonVisionRunnable implements Runnable{
    private final PhotonPoseEstimator[] photonPoseEstimators;
    private final RawSubscriber[] rawSubscribers;
    private final AddVisionMeasurement poseConsumer;
    private final Supplier<Pose2d> poseSupplier;
    // subscribers are components or nodes that receive data from a publisher via a communication channel (called a topic)

    public PhotonVisionRunnable(String[] cameras, Transform3d robotToCamera, AddVisionMeasurement poseConsumer, Supplier<Pose2d> poseSupplier){
        this.poseConsumer = poseConsumer;
        this.poseSupplier = poseSupplier;

        for(int i = 0; i < cameras.length; i++){
            aprilTagPublishers[i] = NetworkTableInstance.getDefault().getStructArrayTopic("ApirlTag-" + cameras[i], new AprilTagStruct()).publish();
        }
    }
    public void run() {      
        // Get AprilTag data
        var emptyAprilTagArray = new AprilTag[0];
    while (!Thread.interrupted()) {
      // Block the thread until new data comes in from PhotonVision
      int[] signaledHandles = null;
      try {
        signaledHandles = WPIUtilJNI.waitForObjects(waitHandles);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
        
        var currentRobotPose = poseSupplier.get();
      for (int i = 0; i < signaledHandles.length; i++) {
        int cameraIndex = getCameraIndex(signaledHandles[i]);
        var aprilTagPublisher = aprilTagPublishers[cameraIndex];
        var photonPoseEstimator = photonPoseEstimators[cameraIndex];

        // Get AprilTag data
        var photonResults = getLatestResult(cameraIndex);
        if (photonResults.hasTargets() && (photonResults.targets.size() > 1
            || (photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD))) {

          // Send the AprilTag(s) to NT for AdvantageScope
          aprilTagPublisher.accept(
              photonResults.targets.stream()
                  .map(
                      target -> getTargetPose(
                          target,
                            currentRobotPose,
                            photonPoseEstimator.getRobotToCameraTransform()))
                  .toArray(AprilTag[]::new));

          photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
}
