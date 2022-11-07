/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRC5010.Vision;

public class VerticalLimeLight extends VisionLimeLight {
  /**
   * Creates a new LimeLightVision.
   */


  //class for when the limelight is mounted "normally" or where the leds are mounted horizontally
  //used when the limelight is mounted horiziontal, landscape
  public VerticalLimeLight(String name, int colIndex) {
    super(name, colIndex);
  }

  public VerticalLimeLight(String name, double camHeight, double camAngle, double targetHeight, int colIndex,String driverTabeName) {
    super(name, camHeight, camAngle, targetHeight, colIndex, driverTabeName);
  }

  @Override
  public void updateViaNetworkTable(String path) {
    VisionValuesLimeLight rawValues = new VisionValuesLimeLight();
    rawValues.setHorizontal(table.getTable(path).getEntry("thor").getDouble(0))
    .setVertical(table.getTable(path).getEntry("tvert").getDouble(0));
    // essential variables from NetworkTables
    updateValues(rawValues,
      () -> table.getTable(path).getEntry("ty").getDouble(0),
      () -> table.getTable(path).getEntry("tx").getDouble(0), 
      () -> table.getTable(path).getEntry("ta").getDouble(0),
      () ->  table.getTable(path).getEntry("tv").getDouble(0) == 1.0,
      () -> table.getTable(path).getEntry("tl").getDouble(0) + 0.011,
      () -> null);
  }
}
