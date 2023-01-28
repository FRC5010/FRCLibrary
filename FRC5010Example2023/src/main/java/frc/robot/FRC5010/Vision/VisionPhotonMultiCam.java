// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import java.security.DrbgParameters.Reseed;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionPhotonMultiCam extends VisionSystem {
  protected List<String> names = new ArrayList<>();
  protected RobotPoseEstimator poseEstimator;
  protected PoseStrategy strategy;
  protected AprilTagFieldLayout fieldLayout;
  protected ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<>();
  protected Pose2d referencePose = new Pose2d();

  /**
   * Creates a new LimeLightVision.
   */

  // makes a new limelight that is vertical, portrait
  public VisionPhotonMultiCam(String name, int colIndex, AprilTagFieldLayout fieldLayout, PoseStrategy strategy) {
    super(name, colIndex);
    this.fieldLayout = fieldLayout; 
    this.strategy = strategy;
  }

  public void addPhotonCamera(String name, Transform3d cameraToRobot) {
    camList.add(new Pair<PhotonCamera, Transform3d>(new PhotonCamera(name), cameraToRobot));
    names.add(name);
    init();
  }

  protected void init() {
    visionLayout.addNumber(name + " FidId", this::getTargetFiducial).withSize(1, 1);
  }

  public void createRobotPoseEstimator() {
    poseEstimator = new RobotPoseEstimator(fieldLayout, strategy, camList);
  }

  @Override
  public void update() {
    rawValues = new VisionValuesPhotonCamera();
    for (int i = 0; i < camList.size(); ++i) {
      updateViaNetworkTable(camList.get(i).getFirst(), camList.get(i).getSecond(), names.get(i));
    }
  }

  public void updateViaNetworkTable(PhotonCamera camera, Transform3d c2rPose, String path) {
    poseEstimator.setReferencePose(referencePose);
    var camResult = camera.getLatestResult();
    Pose2d robotPoseEstInit = new Pose2d();
    Pose3d cameraPose = new Pose3d();
    if (camResult.hasTargets()) {
      var target = camResult.getBestTarget();
      ((VisionValuesPhotonCamera) rawValues).setFiducialId(target.getFiducialId());
      Optional<Pair<Pose3d, Double>> result = poseEstimator.update();
      double deltaTime = Timer.getFPGATimestamp() - result.get().getSecond();

      if (result.isPresent() && result.get() != null && result.get().getFirst() != null) {
        robotPoseEstInit = result.get().getFirst().toPose2d();
      } else {
        robotPoseEstInit = null;
      }
      Pose2d robotPoseEst = robotPoseEstInit;
      referencePose = robotPoseEstInit; 
      updateValues(rawValues,
          () -> target.getYaw(),
          () -> target.getPitch(),
          () -> target.getArea(),
          () -> camResult.hasTargets(),
          () -> deltaTime,
          () -> cameraPose,
          () -> robotPoseEst);
    }

  }

  // name is assigned in the constructor, and will give you the correct limelight
  // table
  // aka use name whenever you use getTable()

  public void setReferencePose(Pose2d previousPose) {
    poseEstimator.setReferencePose(previousPose);
  }

  public int getTargetFiducial() {
    return ((VisionValuesPhotonCamera) rawValues).getFiducialId();
  }

  public void setLight(boolean on) {
    camList.get(0).getFirst().setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void flashLight() {
    camList.get(0).getFirst().setLED(VisionLEDMode.kBlink);
  }

  public boolean isLightOn() {
    return camList.get(0).getFirst().getLEDMode().compareTo(VisionLEDMode.kOn) == 0;
  }

  public void toggleLight() {
    camList.get(0).getFirst().setLED(camList.get(0).getFirst().getLEDMode() == VisionLEDMode.kOff ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setPipeline(int pipeline) {
    camList.get(0).getFirst().setPipelineIndex(pipeline);
  }

  public void setSnapshotMode(int snapVal) {
    camList.get(0).getFirst().takeInputSnapshot();
  }
}
