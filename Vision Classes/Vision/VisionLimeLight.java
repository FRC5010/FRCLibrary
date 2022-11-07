/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRC5010.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FRC5010.VisionSystem;

public class VisionLimeLight extends VisionSystem {
  protected NetworkTableInstance table;

  /**
   * Creates a new LimeLightVision.
   */

   // makes a new limelight that is vertical, portrait
  public VisionLimeLight(String name, int colIndex) {
    super(name, colIndex);
    init();
  }

  public VisionLimeLight(String name, double camHeight, double camAngle, double targetHeight, int colIndex,String driverTabeName) {
    super(name, camHeight, camAngle, targetHeight, colIndex, driverTabeName);
    init();
  }

  protected void init() {
    table = NetworkTableInstance.getDefault();
  }

  public void updateViaNetworkTable(String path) {
    VisionValuesLimeLight rawValues = new VisionValuesLimeLight();
    rawValues.setHorizontal(table.getTable(path).getEntry("thor").getDouble(0))
    .setVertical(table.getTable(path).getEntry("tvert").getDouble(0));

    updateValues(rawValues,
      () -> table.getTable(path).getEntry("tx").getDouble(0),
      () -> table.getTable(path).getEntry("ty").getDouble(0), 
      () -> table.getTable(path).getEntry("ta").getDouble(0),
      () -> table.getTable(path).getEntry("tv").getDouble(0) == 1.0,
      () -> table.getTable(path).getEntry("tl").getDouble(0) + 0.011,
      () -> null);
  }

  //name is assigned in the constructor, and will give you the correct limelight table
  //aka use name whenever you use getTable()

  public void setLight(boolean on) {
    table.getTable(name).getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public void flashLight() {
    table.getTable(name).getEntry("ledMode").setNumber(2);
  }

  public boolean isLightOn() {
    return 1 != table.getTable(name).getEntry("ledMode").getNumber(0).intValue();
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

  public void setPipeline(int pipeline){
    NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(pipeline);
  }
}
