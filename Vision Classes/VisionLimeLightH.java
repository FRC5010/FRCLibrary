/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionLimeLightH extends VisionSystem {
  /**
   * Creates a new LimeLightVision.
   */


  //class for when the limelight is mounted "normally" or where the leds are mounted horizontally
  //used when the limelight is mounted horiziontal, landscape
  public VisionLimeLightH(String name, int colIndex) {
    super(name, colIndex);
  }

  public VisionLimeLightH(String name, double camHeight, double camAngle, double targetHeight, int colIndex) {
    super(name, camHeight, camAngle, targetHeight, colIndex);
  }

  @Override
  public void periodic() {
    updateViaNetworkTable(name);
  }

  public void updateViaNetworkTable(String path) {
    // essential variables from NetworkTables
    boolean valid = table.getTable(path).getEntry("tv").getDouble(0) == 1.0;
    
    if (valid) {
      double angleX = table.getTable(path).getEntry("tx").getDouble(0);
      double angleY = table.getTable(path).getEntry("ty").getDouble(0);
      double area = table.getTable(path).getEntry("ta").getDouble(0);

      // calculating distance
      double distance = (targetHeight - camHeight) / (Math.tan(Math.toRadians(angleY + camAngle)) * Math.cos(Math.toRadians(angleX)));
      rawValues = new VisionValues(valid, 0, 0, angleX, angleY, distance);
      smoothedValues = rawValues;
    } else {
      rawValues = new VisionValues();
      smoothedValues = new VisionValues();
    }
  }

  public void setLight(boolean on) {
    table.getTable(name).getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void flashLight() {
    table.getTable(name).getEntry("ledMode").setNumber(2);
  }

  public boolean isLightOn() {
    return 1 != table.getTable(name).getEntry("ledMode").getNumber(0).intValue();
  }

  @Override
  public void setPipeline(double pipeline) {
    NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(pipeline);
  }

  public void setPiPMode(int streamVal){
    table.getTable(name).getEntry("stream").setNumber(streamVal);
  }

  public void setSnapshotMode(int snapVal){
    table.getTable(name).getEntry("snapshot").setNumber(snapVal);
  }

  public void toggleLight() {
    Number lightVal = table.getTable(name).getEntry("ledMode").getNumber(1);
    table.getTable(name).getEntry("ledMode").setNumber(lightVal.intValue() == 3 ? 1 : 3);
  }
}
