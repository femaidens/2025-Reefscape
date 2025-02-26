package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveConstants.Drivetrain;

import java.util.ArrayList;
import java.util.logging.Logger;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSim {
    private PhotonCameraSim simCam;
    private PhotonCamera realCam;
    private VisionSystemSim simVision;
    private Drive drivetrain;
    private Transform3d cameraTrans;

    public VisionSim() {
        if (Robot.isSimulation()) {
            simVision = new VisionSystemSim("main");
            drivetrain = new Drive();

            simVision.addAprilTags(Constants.VisionConstants.kTagLayout);
            //need to be updated for specific cam
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(70);
            cameraProp.setAvgLatencyMs(30);
            cameraProp.setLatencyStdDevMs(10);

            simCam = new PhotonCameraSim(realCam, cameraProp, 0.05, 20);
            // simVision.addCamera(simCam, Constants.VisionConstants.kRobotToCam);

            simCam.enableDrawWireframe(true);

            double camPitch = Units.degreesToRadians(10); // radians
            double camHeightOffGround = 0.8; // meters
            cameraTrans = new Transform3d(
            new Translation3d(0.0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0));
            simVision.addCamera(simCam, cameraTrans);
            realCam = new PhotonCamera("real camera");
        }
    }
    
    public void simulationPeriodic(Pose2d robotSimPose) {
        simVision.update(robotSimPose);
        Pose2d currentPose = drivetrain.getPose();
        Pose3d current3d = new Pose3d(currentPose);
        simVision.update(currentPose);
        var results = realCam.getLatestResult();
        if (results.hasTargets()) {
            ArrayList<Pose3d> targets = new ArrayList<Pose3d>();
            for(PhotonTrackedTarget t :realCam.getLatestResult().getTargets()) {
                targets.add(current3d.transformBy(cameraTrans).transformBy(t.getBestCameraToTarget()));
            }
            // Logger.recordOutput("photonvision/targetposes", targets.toArray(new Pose3d[targets.size()]));    
        } else {
            // Logger.getInstance().recordOutput("photonvision/targetposes", new Pose3d[] {});
        }
    }

    public void loadAprilTags() {
        try {
            // simVision.addVisionTargets(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
        }
        catch(Exception e) {
            System.out.println("woops can't load the field");
        }
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) simVision.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return simVision.getDebugField();
    }
}