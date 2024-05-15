// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class AprilTags {
  public static AprilTagFieldLayout aprilTagFieldLayout;
  public static AprilTagFieldLayout aprilTagRoomLayout;
  public static Map<Pose2d, Integer> poseToID = new HashMap<>();

  public static enum AprilTag5010 {
    // TODO: Define April Tag positions here.
    ID0(0, 0, 0, 0, 0),
    ID1(0, 0, 0, 0, 0),
    ID2(0, 0, 0, 0, 0),
    ID3(0, 0, 0, 0, 0),
    ID4(0, 0, 0, 0, 0),
    ID5(0, 0, 0, 0, 0),
    ID6(0, 0, 0, 0, 0),
    ID7(0, 0, 0, 0, 0),
    ID8(0, 0, 0, 0, 0),
    ID9(0, 0, 0, 0, 0),
    ID10(0, 0, 0, 0, 0),
    ID11(0, 0, 0, 0, 0),
    ID12(0, 0, 0, 0, 0),
    ID13(0, 0, 0, 0, 0),
    ID14(0, 0, 0, 0, 0),
    ID15(0, 0, 0, 0, 0),
    ID16(0, 0, 0, 0, 0),
    ID17(0, 0, 0, 0, 0),
    ID18(0, 0, 0, 0, 0),
    ID19(0, 0, 0, 0, 0),
    ID20(0, 0, 0, 0, 0),
    ID21(0, 0, 0, 0, 0),
    ID22(0, 0, 0, 0, 0),
    ID23(
        "C102B Door",
        Units.inchesToMeters(274.5),
        Units.inchesToMeters(455.625),
        Units.inchesToMeters(69.75),
        Units.degreesToRadians(180),
        0),
    ID24(
        "Broken TV",
        Units.inchesToMeters(0),
        Units.inchesToMeters(189.625),
        Units.inchesToMeters(69.75),
        0,
        0),
    ID25(
        "Corner Cabinet 2",
        Units.inchesToMeters(36.875),
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(70.875),
        Units.degreesToRadians(90),
        Units.degreesToRadians(0)),
    ID26(
        "Corner Cabinet 1",
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(20.375),
        Units.inchesToMeters(70.875),
        Units.degreesToRadians(0),
        Units.degreesToRadians(0)),
    ID27(
        "Belts Cabinet",
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(116.25),
        Units.inchesToMeters(70.875),
        Units.degreesToRadians(0),
        Units.degreesToRadians(0)),
    ID28("Motor Cabinet", Units.inchesToMeters(14.25), Units.inchesToMeters(0), 0, 0, 0),
    ID29(
        "Tack Board 1",
        Units.inchesToMeters(94.125),
        Units.inchesToMeters(.75),
        Units.inchesToMeters(69.75),
        Units.degreesToRadians(90),
        0);

    // --------------------------------------
    // The rest of this is just boiler-plate for setting up the aprilTagPoses array
    // with all of the defined april tags.
    public Pose3d pose;
    public String fieldDescriptor;

    private AprilTag5010(double xPos, double yPos, double zPos, double pitch, double yaw) {
      this.fieldDescriptor = this.name() + " ID:" + this.ordinal();
      pose =
          new Pose3d(
              new Translation3d(xPos, yPos, zPos),
              new Rotation3d(0.0, Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)));
    }

    private AprilTag5010(
        String fieldDescriptor, double xPos, double yPos, double zPos, double pitch, double yaw) {
      this.fieldDescriptor = fieldDescriptor + " ID:" + this.ordinal();
      pose =
          new Pose3d(
              new Translation3d(xPos, yPos, zPos),
              new Rotation3d(0.0, Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)));
    }
  }

  static {
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

      List<AprilTag> aprilTagPoses = new ArrayList<>();
      for (AprilTag5010 aprilTag : AprilTag5010.values()) {
        aprilTagPoses.add(new AprilTag(aprilTag.ordinal(), aprilTag.pose));
      }

      aprilTagRoomLayout =
          new AprilTagFieldLayout(aprilTagPoses, Units.feetToMeters(30), Units.feetToMeters(30));

    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
  }
}
