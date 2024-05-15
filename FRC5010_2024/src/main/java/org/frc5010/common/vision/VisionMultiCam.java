// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.drive.pose.DrivetrainPoseEstimator;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */
public class VisionMultiCam extends VisionSystem {
  protected List<String> names = new ArrayList<>();
  protected Map<String, VisionSystem> cameras = new HashMap<>();
  protected AprilTagFieldLayout fieldLayout;
  protected DrivetrainPoseEstimator drivetrainPoseEstimator = null;

  /** Creates a new LimeLightVision. */

  // makes a new limelight that is vertical, portrait
  public VisionMultiCam(String name, int colIndex, AprilTagFieldLayout fieldLayout) {
    super(name, colIndex, fieldLayout);
    this.fieldLayout = fieldLayout;
  }

  public void addPhotonCamera(
      String name,
      int colIndex,
      Transform3d cameraToRobot,
      PoseStrategy strategy,
      DrivetrainPoseEstimator drivetrainPoseEstimator) {
    cameras.put(
        name,
        new VisionPhoton(
            name, colIndex, fieldLayout, strategy, drivetrainPoseEstimator, cameraToRobot));
    names.add(name);
    updateValues = true;
  }

  public void addLimeLightCamera(String name, int colIndex, Supplier<GenericGyro> gyro) {
    cameras.put(name, new VisionLimeLight(name, colIndex, fieldLayout, cameraToRobot, gyro));
    names.add(name);
    updateValues = true;
  }

  public void addLimeLightTargetCam(
      String name,
      DoubleSupplier cameraHeight,
      DoubleSupplier cameraAngle,
      double targetHeight,
      int colIndex,
      Transform3d cameraTransform3d) {
    cameras.put(
        name,
        new VisionLimeLightLib(
            name,
            cameraHeight,
            cameraAngle,
            () -> 0,
            targetHeight,
            colIndex,
            fieldLayout,
            "Vision",
            cameraTransform3d));
    names.add(name);
    updateValues = true;
    this.cameraToRobot = cameraTransform3d;
  }

  public Transform3d getCameraPose(String cameraName) {
    return cameras.get(cameraName).getCameraToRobot();
  }

  public VisionSystem getCamera(String name) {
    return cameras.get(name);
  }

  @Override
  public void update() {
    rawValues.clearValues();
    cameras.keySet().stream().forEach(it -> updateFromCamera(cameras.get(it), it));
  }

  public void updateFromCamera(VisionSystem camera, String path) {
    camera.update();
    var camResult = camera.getRawValues();
    updateBatchValues(
        camResult.getAngleX(),
        camResult.getAngleY(),
        camResult.getDistance(),
        camResult.getArea(),
        camResult.getValid(),
        camResult.getLatencies(),
        camResult.getFiducialIds(),
        camResult.getTargetVectors(),
        camResult.getRobotPoses(),
        camResult.getPoseDistances());
  }

  protected void updateBatchValues(
      Double angleXSup,
      Double angleYSup,
      Double distanceSup,
      Double areaSup,
      Boolean validSup,
      Map<String, Double> latencySup,
      Map<String, Integer> fiducialID,
      Map<String, Pose3d> cameraPoseSupplier,
      Map<String, Pose2d> robotPoseSupplier,
      Map<String, Double> poseDistanceSup) {
    boolean valid = validSup;
    if (valid) {
      rawValues
          .setValid(rawValues.valid || valid)
          .setDistance(distanceSup)
          .setYaw(angleXSup)
          .setPitch(angleYSup)
          .addPoseDistances(poseDistanceSup)
          .addLatencies(latencySup)
          .addFiducialIds(fiducialID)
          .addTargetVectors(cameraPoseSupplier)
          .addRobotPoses(robotPoseSupplier);
      if (isSmoothingValues) {
        smoothedValues.averageValues(rawValues, 5);
      } else {
        smoothedValues.storeValues(rawValues, 5);
      }
    } else {
      // smoothedValues.deprecateValues();
    }
  }

  public DrivetrainPoseEstimator getDrivetrainPoseEstimator() {
    return drivetrainPoseEstimator;
  }

  public void setDrivetrainPoseEstimator(DrivetrainPoseEstimator drivetrainPoseEstimator) {
    this.drivetrainPoseEstimator = drivetrainPoseEstimator;
  }

  // name is assigned in the constructor, and will give you the correct limelight
  // table
  // aka use name whenever you use getTable()

  public int getTargetFiducial() {
    return rawValues.getFiducialId();
  }

  public void setLight(boolean on) {
    cameras.values().stream().forEach(it -> it.setLight(on));
  }

  public void flashLight() {
    cameras.values().stream().forEach(it -> it.flashLight());
  }

  public boolean isLightOn() {
    return cameras.values().stream().map(it -> isLightOn()).reduce((a, b) -> a || b).orElse(false);
  }

  public void setPipeline(int pipeline) {
    cameras.values().stream().forEach(it -> it.setPipeline(pipeline));
  }

  public void setSnapshotMode(int snapVal) {
    cameras.values().stream().forEach(it -> it.setSnapshotMode(snapVal));
  }

  public void setLight(String name, boolean on) {
    cameras.get(name).setLight(on);
  }

  public void flashLight(String name) {
    cameras.get(name).flashLight();
  }

  public boolean isLightOn(String name) {
    return cameras.get(name).isLightOn();
  }

  public void setPipeline(String name, int pipeline) {
    cameras.get(name).setPipeline(pipeline);
  }

  public void setSnapshotMode(String name, int snapVal) {
    cameras.get(name).setSnapshotMode(snapVal);
  }

  public Transform3d getCameraToRobot(String name) {
    return cameras.get(name).getCameraToRobot();
  }
}
