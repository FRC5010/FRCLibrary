/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRC5010.Vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Add your docs here.
 */
public class VisionValues {

    // the essential variables, stuff they already give me
    // TODO: If you ever get rid of a networkentry variable in opensight, update
    // vision classes

    protected boolean valid = false;
    protected double latency = 0.0;
    protected double angleX = 0.0;
    protected double angleY = 0.0;
    // distance
    protected double distance = 0.0;
    
    protected double area = 0.0;
    protected List<Pose3d> cameraToTarget = new ArrayList<>();
    protected List<Pose2d> robotPoses = new ArrayList<>();

    protected int count = 0;

    public VisionValues() {        
    }

    public void averageValues(VisionValues rawValues, int maxCount) {
        count++;
        count = Math.min(count, maxCount);
        valid = count >= maxCount;
        angleX = ((count - 1) * angleX + rawValues.getAngleX()) / count;
        angleY = ((count - 1) * angleY + rawValues.getAngleY()) / count;
        distance = ((count - 1) * distance + rawValues.getDistance()) / count;
    }

    public boolean getValid() {
        return valid;
    }

    public double getAngleX() {
        return angleX;
    }

    public double getAngleY() {
        return angleY;
    }

    public double getDistance() {
        return distance;
    }
    
    public double getDistanceViaArea() {
        return 0;
    }

    public double getArea() {
        return area;
    }
    
    public double getLatency() {
        return latency;
    }

    public VisionValues setArea(double area) {
        this.area = area;
        return this;
    }

    public VisionValues setPitch(double pitch) {
        this.angleY = pitch;
        return this;
    }

    public VisionValues setYaw(double yaw) {
        this.angleX = yaw;
        return this;
    }

    public VisionValues setDistance(double distance) {
        this.distance = distance;
        return this;
    }

    public VisionValues setLatency(double latency) {
        this.latency = latency;
        return this;
    }

    public VisionValues setValid(boolean valid) {
        this.valid = valid;
        return this;
    }

    public Pose3d getCameraPose() {
        return cameraToTarget.size() > 0? cameraToTarget.get(0) : new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    } 

    public VisionValues addCameraPose(Pose3d camPose) {
        cameraToTarget.add(camPose);
        return this;
    }

    public VisionValues addCameraPoses(List<Pose3d> camPose) {
        cameraToTarget.addAll(camPose);
        return this;
    }

    public Pose3d getCameraToTarget(int i) {
        Pose3d pose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        if (i < cameraToTarget.size()) {
            if (null != cameraToTarget.get(i)) {
                pose = cameraToTarget.get(i);
            }
        }
        return pose;
    }

    public Pose2d getRobotPose() {
        return robotPoses.size() > 0? robotPoses.get(0) : new Pose2d(0, 0, new Rotation2d(0, 0));
    } 

    public VisionValues addRobotPose(Pose2d robotPose) {
        robotPoses.add(robotPose);
        return this;
    }

    public VisionValues addRobotPoses(List<Pose2d> robotPose) {
        robotPoses.addAll(robotPose);
        return this;
    }

    public Pose2d getRobotPose(int i) {
        Pose2d pose = new Pose2d(0, 0, new Rotation2d(0, 0));
        if (i < robotPoses.size()) {
            if (null != robotPoses.get(i)) {
                pose = robotPoses.get(i);
            }
        }
        return pose;
    }

    public List<Pose2d> getRobotPoses() {
        return robotPoses;
    }

}
