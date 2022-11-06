/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRC5010.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FRC5010.VisionSystem;

public class VisionPhotonCamera extends VisionSystem {
  protected PhotonCamera camera;

  /**
   * Creates a new LimeLightVision.
   */

   // makes a new limelight that is vertical, portrait
  public VisionPhotonCamera(String name, int colIndex) {
    super(name, colIndex);
    init();
  }

  public VisionPhotonCamera(String name, double camHeight, double camAngle, double targetHeight, int colIndex,String driverTabeName) {
    super(name, camHeight, camAngle, targetHeight, colIndex, driverTabeName);
    init();
  }

  protected void init() {
    camera = new PhotonCamera(name);
  }

  public void updateViaNetworkTable(String path) {
    VisionValuesPhotonCamera rawValues = new VisionValuesPhotonCamera();
    var result = camera.getLatestResult();
    var target = result.getBestTarget();
    rawValues.setFiducialId(target.getFiducialId());
    target.getCameraToTarget();
    updateValues(rawValues,
      () -> target.getYaw(),
      () -> target.getPitch(), 
      () -> target.getArea(),
      () -> result.hasTargets(),
      () -> result.getLatencyMillis());
  }

  //name is assigned in the constructor, and will give you the correct limelight table
  //aka use name whenever you use getTable()

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
    camera.setLED(camera.getLEDMode() == VisionLEDMode.kOff ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setPipeline(int pipeline){
    camera.setPipelineIndex(pipeline);
  }

  public void setSnapshotMode(int snapVal) {
    camera.takeInputSnapshot();
  }
}
