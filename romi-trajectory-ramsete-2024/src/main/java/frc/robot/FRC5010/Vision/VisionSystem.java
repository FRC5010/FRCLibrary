/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRC5010.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// base vision code, can be extended to more specific vision systems.
public abstract class VisionSystem extends SubsystemBase {

  protected String name;
  protected double camHeight, camAngle, targetHeight;
  protected VisionValues rawValues, smoothedValues;
  protected ShuffleboardLayout visionLayout;
  protected boolean updateValues = false;
  protected double CAMERA_CAL_ANGLE = 0;
  protected ShuffleboardLayout driverLayout;
  protected VisionConstants constants;
  protected AprilTagFieldLayout fieldLayout;
  protected boolean smoothValues = false;

  // variables needed to process new variables, plus the new variables
  // angles

  // giant list of values from camera "table" and is saved into vision values
  public VisionSystem(String name, int colIndex, AprilTagFieldLayout fieldLayout) {
    this.name = name;
    rawValues = new VisionValues();
    smoothedValues = new VisionValues();
    constants = new VisionConstants();
    this.fieldLayout = fieldLayout;
    ShuffleboardTab driverTab = Shuffleboard.getTab(VisionConstants.SBTabVisionDisplay);
    visionLayout = driverTab.getLayout(name + " Vision", BuiltInLayouts.kGrid)
        .withPosition(colIndex, 0).withSize(2, 5);
    visionLayout.addBoolean("Has Target", this::isValidTarget);
    visionLayout.addNumber("Distance", this::getDistance).withSize(1, 1);
    visionLayout.addNumber("Fiducial", () -> getRawValues().getFiducialId());
    visionLayout.addNumber("Robot Pose X", () -> null != smoothedValues.getRobotPose() ? getRawValues().getRobotPose().getX() : -1);
    visionLayout.addNumber("Robot Pose Y", () -> null != smoothedValues.getRobotPose() ? getRawValues().getRobotPose().getY() : -1);
    visionLayout.addNumber("Target Pose X", () -> null != smoothedValues.getTargetVector() ? getRawValues().getTargetVector().getX() : -1);
    visionLayout.addNumber("Target Pose Y", () -> null != smoothedValues.getTargetVector() ? getRawValues().getTargetVector().getY() : -1);
  }

  // more specific values to define the camera
  public VisionSystem(String name, double camHeight, double camAngle, double targetHeight, int colIndex,
      String driverTabeName) {
    this.name = name;
    rawValues = new VisionValues();
    smoothedValues = new VisionValues();
    this.camHeight = camHeight;
    this.camAngle = camAngle;
    this.targetHeight = targetHeight;
    constants = new VisionConstants();
    CAMERA_CAL_ANGLE = Math.toDegrees(Math.tanh((targetHeight - camHeight) / VisionConstants.CAMERA_CAL_DISTANCE));
    ShuffleboardTab visionTab = Shuffleboard.getTab(VisionConstants.SBTabVisionDisplay);
    visionLayout = visionTab.getLayout(name + " Vision", BuiltInLayouts.kGrid)
        .withPosition(colIndex, 0).withSize(1, 5);

    visionLayout.addBoolean(name + " Has Target", this::isValidTarget).withSize(1, 1);
    visionLayout.addNumber(name + " Distance", this::getDistance).withSize(1, 1);
    visionLayout.addNumber(name + " X Angle", this::getAngleX).withSize(1, 1);
    visionLayout.addNumber(name + " Y Angle", this::getAngleY).withSize(1, 1);
    // visionLayout.addNumber(name + " Horizontal", this::getHorizontal).withSize(1,
    // 1);
    // visionLayout.addNumber(name + " Vertical", this::getVertical).withSize(1, 1);
    // visionLayout.addNumber(name + " Cal Angle", this::getCalAngle).withSize(1,
    // 1);
    // visionLayout.addBoolean(name + " Driver Mode",
    // RobotContainer::getDriverMode).withSize(1, 1);

    ShuffleboardTab driverTab = Shuffleboard.getTab(driverTabeName);
    driverLayout = driverTab.getLayout(name + " Vision", BuiltInLayouts.kGrid)
        .withPosition(colIndex + 1, 0).withSize(1, 1);
    driverLayout.addBoolean("Limelight On", this::isLightOn);
  }

  public abstract void setPipeline(int pipeline);

  public abstract void setLight(boolean on);

  public abstract boolean isLightOn();

  public abstract void flashLight();

  public abstract void setSnapshotMode(int snapVal);

  public abstract void update();

  public VisionValues getRawValues() {
    return rawValues;
  }

  public void setUpdateValues(boolean update) {
    updateValues = update;
  }

  @Override
  public void periodic() {
    if (updateValues) {
      update();
    }
  }

  protected void updateValues(VisionValues rawValues,
      DoubleSupplier angleXSup, DoubleSupplier angleYSup,
      DoubleSupplier areaSup, BooleanSupplier validSup, DoubleSupplier latencySup,
      IntSupplier fidSup,
      Supplier<Pose3d> cameraPoseSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    boolean valid = validSup.getAsBoolean();
    if (valid) {
      // calculating distance
      double angleX = angleXSup.getAsDouble();
      double angleY = angleYSup.getAsDouble();
      double distance = 0;
      if (null != cameraPoseSupplier.get() && null != robotPoseSupplier.get()) {
        Pose3d targetPose = cameraPoseSupplier.get();
        Pose2d robotPose = robotPoseSupplier.get();
        Translation3d targetTrans = new Translation3d(targetPose.getX(), targetPose.getY(), targetPose.getZ());
        Translation2d robotTrans = new Translation2d(robotPose.getX(), robotPose.getY());
        distance = robotTrans.getDistance(targetTrans.toTranslation2d());
      } else if (angleX != 0 || angleY != 0) {
        distance = (targetHeight - camHeight)
            / (Math.tan(Math.toRadians(angleY + camAngle)) * Math.cos(Math.toRadians(angleX)));
            if (areaSup.getAsDouble() != 0) {
              // This exact calculation would need to be determined on a camera by camera basis
              double areaDistance = areaSup.getAsDouble() * 0; // conversion factor
              distance = (distance * 1.0 + areaDistance * 0.0); // weighting factors
            }    
      }
      this.rawValues = rawValues;
      rawValues
          .setValid(valid)
          .addLatency(name, latencySup.getAsDouble())
          .setYaw(angleX)
          .setPitch(angleY)
          .setArea(areaSup.getAsDouble())
          .setDistance(distance)
          .addFiducialId(name, fidSup.getAsInt())
          .addTargetVector(name, cameraPoseSupplier.get())
          .addRobotPose(name, robotPoseSupplier.get());
      if (smoothValues) {    
        smoothedValues.averageValues(rawValues, 5);
      } else {
        smoothedValues.storeValues(rawValues, 5);
      }
    } else {
      smoothedValues.deprecateValues();
    }
  }

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
    return smoothValues ? smoothedValues.getDistance() : getRawDistance();
  }

  public double getAngleX() {
    return smoothValues ? smoothedValues.getAngleX() : getRawAngleX();
  }

  public double getAngleY() {
    return smoothValues ? smoothedValues.getAngleY() : getRawAngleY();
  }

  public double getCalAngle() {
    return CAMERA_CAL_ANGLE;
  }

  public boolean isRawValidTarget() {
    return rawValues.getValid();
  }

  public boolean isValidTarget() {
    return smoothValues ? smoothedValues.getValid() : isRawValidTarget();
  }

  public void setValueSmooting(boolean smooth) {
    smoothValues = smooth;
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public void calibarateCamAngle(double angleY) {
    camAngle = CAMERA_CAL_ANGLE - angleY;
  }
}
