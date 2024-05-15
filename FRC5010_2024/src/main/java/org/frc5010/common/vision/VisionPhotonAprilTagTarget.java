// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionPhotonAprilTagTarget extends VisionSystem {
  protected PhotonCamera camera;
  protected int targetTagId = 0;

  /** Creates a new LimeLightVision. */

  // makes a new limelight that is vertical, portrait
  public VisionPhotonAprilTagTarget(
      String name,
      DoubleSupplier camHeight,
      DoubleSupplier camAngle,
      DoubleSupplier cameraDistance,
      double targetHeight,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      String driverTabName) {
    super(
        name,
        camHeight,
        camAngle,
        cameraDistance,
        targetHeight,
        colIndex,
        fieldLayout,
        driverTabName);
    this.fieldLayout = fieldLayout;
    camera = new PhotonCamera(name);

    visionLayout.addNumber(name + " Height", () -> camHeight.getAsDouble()).withSize(1, 1);
    visionLayout.addNumber(name + " Angle", () -> camAngle.getAsDouble()).withSize(1, 1);
    visionLayout.addNumber(name + " XPos", () -> cameraDistance.getAsDouble()).withSize(1, 1);
  }

  public void setTargetTagId(int targetTagId) {
    this.targetTagId = targetTagId;
  }

  public int getTargetTagId() {
    return targetTagId;
  }

  @Override
  public void update() {
    rawValues.clearValues();
    updateViaNetworkTable(camera, name);
  }

  public void updateViaNetworkTable(PhotonCamera camera, String path) {
    var camResult = camera.getLatestResult();
    List<PhotonTrackedTarget> targetInit = null;
    double deltaTimeInit = Timer.getFPGATimestamp() - (camResult.getLatencyMillis() / 1000.0);
    if (camResult.hasTargets()) {
      targetInit = camResult.getTargets();
      Optional<PhotonTrackedTarget> target =
          targetInit.stream().filter(it -> it.getFiducialId() == targetTagId).findFirst();
      double deltaTime = deltaTimeInit;
      if (target.isPresent()) {
        updateValues(
            rawValues,
            () -> target.get().getYaw(),
            () -> target.get().getPitch(),
            () -> target.get().getArea(),
            () -> camResult.hasTargets(),
            () -> deltaTime,
            () -> target.get().getFiducialId(),
            () -> fieldLayout.getTagPose(target.get().getFiducialId()).orElse(null),
            () -> null);
      } else {
        updateValues(
            rawValues,
            () -> 0,
            () -> 0,
            () -> 0,
            () -> false,
            () -> 0,
            () -> 0,
            () -> null,
            () -> null);
      }
    } else {
      updateValues(
          rawValues,
          () -> 0,
          () -> 0,
          () -> 0,
          () -> false,
          () -> 0,
          () -> 0,
          () -> null,
          () -> null);
    }
  }

  // name is assigned in the constructor, and will give you the correct limelight
  // table
  // aka use name whenever you use getTable()

  public int getTargetFiducial() {
    return rawValues.getFiducialId();
  }

  public void setLight(boolean on) {
    camera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void flashLight() {
    camera.setLED(VisionLEDMode.kBlink);
  }

  public boolean isLightOn() {
    return camera.getLEDMode().compareTo(VisionLEDMode.kOn) == 0;
  }

  public void toggleLight() {
    camera.setLED(
        camera.getLEDMode() == VisionLEDMode.kOff ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  public void setSnapshotMode(int snapVal) {
    camera.takeInputSnapshot();
  }
}
