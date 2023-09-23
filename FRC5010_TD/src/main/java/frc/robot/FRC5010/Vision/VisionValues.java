/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRC5010.Vision;

import java.util.HashMap;
import java.util.Map;

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

    protected Boolean valid = false;
    protected Map<String, Double> latencies = new HashMap<>();
    protected Double angleX = 0.0;
    protected Double angleY = 0.0;
    // distance
    protected Double distance = 0.0;

    protected Double area = 0.0;
    protected Map<String, Pose3d> robotToTarget = new HashMap<>();
    protected Map<String, Pose2d> robotPoses = new HashMap<>();

    protected int count = 0;
    private int fiducialId = 0;

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

    public Boolean getValid() {
        return valid;
    }

    public Double getAngleX() {
        return angleX;
    }

    public Double getAngleY() {
        return angleY;
    }

    public Double getDistance() {
        return distance;
    }

    public Double getDistanceViaArea() {
        return 0.0;
    }

    public Double getArea() {
        return area;
    }

    public Double getLatency(String camera) {
        return latencies.get(camera);
    }

    public VisionValues setArea(Double area) {
        this.area = area;
        return this;
    }

    public VisionValues setPitch(Double pitch) {
        this.angleY = pitch;
        return this;
    }

    public VisionValues setYaw(Double yaw) {
        this.angleX = yaw;
        return this;
    }

    public VisionValues setDistance(Double distance) {
        this.distance = distance;
        return this;
    }

    public VisionValues addLatency(String camera, Double latency) {
        latencies.put(camera, latency);
        return this;
    }

    public VisionValues setValid(Boolean valid) {
        this.valid = valid;
        return this;
    }

    public int getFiducialId() {
        return fiducialId;
    }

    public VisionValues setFiducialId(int id) {
        fiducialId = id;
        return this;
    }

    public Pose3d getTargetVector() {
        return robotToTarget.size() > 0 ? robotToTarget.values().iterator().next()
                : new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    }

    public VisionValues addTargetVector(String camera, Pose3d camPose) {
        robotToTarget.put(camera, camPose);
        return this;
    }

    public VisionValues addTargetVectors(Map<String, Pose3d> camPoses) {
        robotToTarget.putAll(camPoses);
        return this;
    }

    public Map<String, Pose3d> getTargetVectors() {
        return robotToTarget;
    }

    public Pose3d getRobotToTarget(String camera) {
        Pose3d pose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        if (robotToTarget.containsKey(camera)) {
            if (null != robotToTarget.get(camera)) {
                pose = robotToTarget.get(camera);
            }
        }
        return pose;
    }

    public Pose2d getRobotPose() {
        return robotPoses.size() > 0 ? robotPoses.values().iterator().next()
                : new Pose2d(0, 0, new Rotation2d(0, 0));
    }

    public VisionValues addRobotPose(String camera, Pose2d robotPose) {
        robotPoses.put(camera, robotPose);
        return this;
    }

    public VisionValues addRobotPoses(Map<String, Pose2d> robotPoses) {
        robotPoses.putAll(robotPoses);
        return this;
    }

    public Pose2d getRobotPose(String camera) {
        Pose2d pose = new Pose2d(0, 0, new Rotation2d(0, 0));
        if (robotPoses.containsKey(camera)) {
            if (null != robotPoses.get(camera)) {
                pose = robotPoses.get(camera);
            }
        }
        return pose;
    }

    public Map<String, Pose2d> getRobotPoses() {
        return robotPoses;
    }

}
