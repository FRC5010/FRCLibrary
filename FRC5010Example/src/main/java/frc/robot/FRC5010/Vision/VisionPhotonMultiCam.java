// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FRC5010.VisionSystem;

/** Add your docs here. */
public class VisionPhotonMultiCam extends VisionSystem {
    protected List<PhotonCamera> cameras = new ArrayList<>();
    protected List<Transform3d> poses = new ArrayList<>();
    protected List<String> names = new ArrayList<>();
    /**
     * Creates a new LimeLightVision.
     */
  
    // makes a new limelight that is vertical, portrait
    public VisionPhotonMultiCam(String name, int colIndex) {
      super(name, colIndex);
    }
  
    public void addPhotonCamera(String name, Transform3d cameraToRobot) {
      cameras.add(new PhotonCamera(name));
      names.add(name);
      poses.add(cameraToRobot);  
      init();
    }
  
    protected void init() {
      visionLayout.addNumber(name + " FidId", this::getTargetFiducial).withSize(1, 1);
    }
  
    @Override
    public void update() {
        for(int i = 0; i < cameras.size(); ++i) {
            updateViaNetworkTable(cameras.get(i), poses.get(i), names.get(i));
        }
    }

    public void updateViaNetworkTable(PhotonCamera camera, Transform3d c2rPose, String path) {
      rawValues = new VisionValuesPhotonCamera();
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        var target = result.getBestTarget();
        ((VisionValuesPhotonCamera) rawValues).setFiducialId(target.getFiducialId());
        Pose3d camPose = null;
        for (PhotonTrackedTarget photonTgt : result.getTargets()) {
          if (0.2 > photonTgt.getPoseAmbiguity() || -1 != photonTgt.getPoseAmbiguity()) {
            Transform3d cam2Tgt = photonTgt.getBestCameraToTarget();
            int fiducialId = photonTgt.getFiducialId();
            if (fiducialId >= 0 && fiducialId < AprilTags.aprilTagPoses.size()) {
              Pose3d camPoseTmp = AprilTags.aprilTagPoses.get(fiducialId).pose.transformBy(cam2Tgt.inverse());
              if (null == camPose) {
                camPose = camPoseTmp;
              } else {
                camPose = camPose.transformBy(camPose.minus(camPoseTmp).times(0.5));
              }
            }
          }
        }
        Pose3d cameraPose = camPose;
        Pose2d robotPoseEst = camPose.transformBy(c2rPose).toPose2d();
  
        updateValues(rawValues,
            () -> target.getYaw(),
            () -> target.getPitch(),
            () -> target.getArea(),
            () -> result.hasTargets(),
            () -> Timer.getFPGATimestamp() - result.getLatencyMillis() / 1000.0,
            () -> cameraPose, 
            () -> robotPoseEst);
      }
    }
  
    // name is assigned in the constructor, and will give you the correct limelight
    // table
    // aka use name whenever you use getTable()
  
    public int getTargetFiducial() {
      return ((VisionValuesPhotonCamera) rawValues).getFiducialId();
    }
  
    public void setLight(boolean on) {
      cameras.get(0).setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }
  
    public void flashLight() {
        cameras.get(0).setLED(VisionLEDMode.kBlink);
    }
  
    public boolean isLightOn() {
      return cameras.get(0).getLEDMode().compareTo(VisionLEDMode.kOn) == 0;
    }
  
    public void toggleLight() {
        cameras.get(0).setLED(cameras.get(0).getLEDMode() == VisionLEDMode.kOff ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }
  
    public void setPipeline(int pipeline) {
        cameras.get(0).setPipelineIndex(pipeline);
    }
  
    public void setSnapshotMode(int snapVal) {
        cameras.get(0).takeInputSnapshot();
    }
  }
