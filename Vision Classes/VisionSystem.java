/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;

// base vision code, can be extended to more specific vision systems.
public abstract class VisionSystem extends SubsystemBase {

  protected String name;
  protected double camHeight, camAngle, targetHeight;
  protected NetworkTableInstance table;
  protected VisionValues rawValues, smoothedValues;
  protected ShuffleboardLayout visionLayout;
  protected boolean updateValues = false;
  protected double CAMERA_CAL_ANGLE = 0;
  protected ShuffleboardLayout driverLayout;
  // variables needed to process new variables, plus the new variables
  // angles


  //giant list of values from camera "table" and is saved into vision values
  public VisionSystem(String name, int colIndex) {
    table = NetworkTableInstance.getDefault();
    this.name = name;
    rawValues = new VisionValues();
    smoothedValues = new VisionValues();
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabVisionDisplay);
    visionLayout = driverTab.getLayout(name + " Vision", BuiltInLayouts.kGrid).withPosition(colIndex, 0).withSize(3, 5);
  }

  //more specific values to define the camera
  public VisionSystem(String name, double camHeight, double camAngle, double targetHeight, int colIndex) {
    this.name = name;
    rawValues = new VisionValues();
    smoothedValues = new VisionValues();
    this.camHeight = camHeight;
    this.camAngle = camAngle;
    this.targetHeight = targetHeight;
    updateValues = true;
    CAMERA_CAL_ANGLE = Math.toDegrees(Math.tanh((targetHeight - camHeight) / VisionConstants.CAMERA_CAL_DISTANCE));
    ShuffleboardTab visionTab = Shuffleboard.getTab(ControlConstants.SBTabVisionDisplay);
    visionLayout = visionTab.getLayout(name + " Vision", BuiltInLayouts.kGrid).withPosition(colIndex, 0).withSize(3, 5);
    table = NetworkTableInstance.getDefault();


    visionLayout.addNumber(name + " Distance", this::getDistance).withSize(1, 1);
    visionLayout.addBoolean(name + " Has Target", this::isValidTarget).withSize(1, 1);
    visionLayout.addNumber(name + " X Angle", this::getAngleX).withSize(1, 1);
    visionLayout.addNumber(name + " Y Angle", this::getAngleY).withSize(1, 1);
    //visionLayout.addNumber(name + " Horizontal", this::getHorizontal).withSize(1, 1);
    //visionLayout.addNumber(name + " Vertical", this::getVertical).withSize(1, 1);
    //visionLayout.addNumber(name + " Cal Angle", this::getCalAngle).withSize(1, 1);
    //visionLayout.addBoolean(name + " Driver Mode", RobotContainer::getDriverMode).withSize(1, 1);

    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    driverLayout = driverTab.getLayout(name + " Vision", BuiltInLayouts.kGrid).withPosition(colIndex + 1, 0).withSize(1, 1);
    driverLayout.addBoolean("Limelight On", this::isLightOn);
    
    
  }
  public abstract void setPipeline(double pipeline);

  public VisionValues getRawValues() {
    return rawValues;
  }

  public abstract void updateViaNetworkTable(String path);

  public abstract void setLight(boolean on);
  public abstract boolean isLightOn();
  public abstract void flashLight();
  public abstract void setSnapshotMode(int snapVal);

  public void setCamAngle(double a) {
    camAngle = a;
  }

  public double getCamAngle() {
    return camAngle;
  }

  public double getRawDistance() {
    return rawValues.getDistance();
  }

  public double getRawAngleX() {
    return rawValues.getAngleX();
  }

  public double getRawAngleY() {
    return rawValues.getAngleY();
  }

  public double getDistance() {
    return smoothedValues.getDistance();
  }

  public double getAngleX() {
    return smoothedValues.getAngleX();
  }

  public double getAngleY() {
    return smoothedValues.getAngleY();
  }

  public double getHorizontal() {
    return smoothedValues.getHorizontal();
  }

  public double getVertical() {
    return smoothedValues.getVertical();
  }

  public double getRawHorizontal() {
    return rawValues.getHorizontal();
  }

  public double getRawVertical() {
    return rawValues.getVertical();
  }

  public double getCalAngle() {
    return CAMERA_CAL_ANGLE;
  }

  public boolean isRawValidTarget() {
    return rawValues.getValid();
  }
  
  public boolean isValidTarget() {
    return smoothedValues.getValid();
  }

  public void calibarateCamAngle(double angleY) {
    camAngle = CAMERA_CAL_ANGLE - angleY;
  }
}
