// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionPhotonMultiCam extends VisionSystem {
  protected List<String> names = new ArrayList<>();
  protected List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
  protected List<PhotonCamera> cameras = new ArrayList<>();
  protected PoseStrategy strategy;
  protected AprilTagFieldLayout fieldLayout;
  protected Pose2d referencePose = new Pose2d(0, 0, new Rotation2d(0));

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
    cameras.add(new PhotonCamera(name));
    names.add(name);
    poseEstimators.add(
      new PhotonPoseEstimator(fieldLayout, strategy, cameras.get(cameras.size() - 1), cameraToRobot));
    init(name);
  }

  protected void init(String name) {
      visionLayout.addNumber(name + " FidId", this::getTargetFiducial).withSize(1, 1);
  }

  @Override
  public void update() {
    rawValues = new VisionValuesPhotonCamera();
    for (int i = 0; i < cameras.size(); ++i) {
      updateViaNetworkTable(cameras.get(i), poseEstimators.get(i), names.get(i));
    }
  }

  public void updateViaNetworkTable(PhotonCamera camera, PhotonPoseEstimator poseEstimator, String path) {
    poseEstimator.setReferencePose(referencePose);
    var camResult = camera.getLatestResult();
    Pose2d robotPoseEstInit = new Pose2d();
    Pose3d trackedTargetPose = new Pose3d();
    var targetInit = new PhotonTrackedTarget();
    double deltaTimeInit = Timer.getFPGATimestamp() - (camResult.getLatencyMillis() / 1000.0);
    if (camResult.hasTargets()) {
      targetInit = camResult.getBestTarget();
      ((VisionValuesPhotonCamera) rawValues).setFiducialId(targetInit.getFiducialId());
      Optional<EstimatedRobotPose> result = poseEstimator.update();

      if (result.isPresent() && result.get().estimatedPose != null) {
        robotPoseEstInit = result.get().estimatedPose.toPose2d();
        deltaTimeInit = result.get().timestampSeconds;
        referencePose = robotPoseEstInit;
      } else {
        robotPoseEstInit = null;
      }
    }
    Pose2d robotPoseEst = robotPoseEstInit;
    double deltaTime = deltaTimeInit;
    var target = targetInit;
    updateValues(rawValues,
        () -> target.getYaw(),
        () -> target.getPitch(),
        () -> target.getArea(),
        () -> camResult.hasTargets(),
        () -> deltaTime,
        () -> fieldLayout.getTagPose(target.getFiducialId()).orElse(trackedTargetPose),
        () -> robotPoseEst);
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
