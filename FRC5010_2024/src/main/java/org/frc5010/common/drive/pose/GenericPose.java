// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5010.common.sensors.gyro.GenericGyro;

/** Add your docs here. */
public abstract class GenericPose {
  protected Field2d field2d;
  protected Matrix<N5, N1> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
  protected Matrix<N3, N1> localMeasurementStdDevs =
      VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  protected Matrix<N3, N1> visionMeasurementStdDevs =
      VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

  protected GenericGyro gyro;

  public GenericPose(GenericGyro gyro) {
    this.gyro = gyro;
  }

  public abstract void resetEncoders();

  public Field2d getField() {
    if (null == field2d) {
      field2d = new Field2d();
    }
    return field2d;
  }

  public double getAccelX() {
    // TODO Auto-generated method stub
    return 0;
  }

  public double getAccelY() {
    // TODO Auto-generated method stub
    return 0;
  }

  public double getAccelZ() {
    // TODO Auto-generated method stub
    return 0;
  }

  public double getGyroAngleX() {
    return gyro.getAngleX();
  }

  public double getGyroAngleY() {
    return gyro.getAngleY();
  }

  public double getGyroAngleZ() {
    // System.out.println(gyro.getAngleZ());
    return gyro.getAngleZ();
  }

  public Rotation2d getGyroRotation2d() {
    // return new Rotation3d(getGyroAngleX(), getGyroAngleY(), getGyroAngleZ()).toRotation2d();
    double angleZ = getGyroAngleZ();
    double radianZ = Math.toRadians(angleZ);
    Rotation2d rotation2d = new Rotation2d(radianZ);
    SmartDashboard.putNumber("Angle Gyro from Pose", rotation2d.getDegrees());
    return rotation2d;
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }
  ;

  public abstract void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Vector<N3> stdVector);

  public abstract void updateLocalMeasurements();

  public abstract Pose2d getCurrentPose();

  public abstract void resetToPose(Pose2d pose);
}
