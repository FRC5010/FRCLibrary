// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FRC5010.Vision.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class VisionLimeLightLib extends VisionSystem {

    public VisionLimeLightLib(String name, int colIndex, AprilTagFieldLayout fieldLayout) {
        super("limelight-" + name, colIndex, fieldLayout);
        init();
    }

    public VisionLimeLightLib(String name, double camHeight, double camAngle, double targetHeight, int colIndex,
            String driverTabeName) {
        super(name, camHeight, camAngle, targetHeight, colIndex, driverTabeName);
    }

    protected void init() {
    }

    public void update() {
        updateViaNetworkTable(name);
    }

    private void updateViaNetworkTable(String name) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (results.targetingResults.valid) {
            VisionValuesLimeLight rawValues = new VisionValuesLimeLight();
            Pose2d robotPose2d = LimelightHelpers.getBotPose2d(name);
            updateValues(rawValues,
                    () -> LimelightHelpers.getTX(name),
                    () -> LimelightHelpers.getTY(name),
                    () -> LimelightHelpers.getTA(name),
                    () -> LimelightHelpers.getTV(name),
                    () -> Timer.getFPGATimestamp() -
                            ((LimelightHelpers.getLatency_Pipeline(name) +
                                    LimelightHelpers.getLatency_Capture(name)) * 0.001),
                    () -> Double.valueOf(LimelightHelpers.getFiducialID(name)).intValue(),
                    () -> fieldLayout.getTagPose(rawValues.getFiducialId()).orElse(null),
                    () -> robotPose2d);
        }
    }

    @Override
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(name, pipeline);
    }

    @Override
    public void setLight(boolean on) {
        if (on)
            LimelightHelpers.setLEDMode_ForceOn(name);
        else
            LimelightHelpers.setLEDMode_ForceOff(name);
    }

    @Override
    public boolean isLightOn() {
        return false;
    }

    public void toggleLight() {
        if (isLightOn())
            setLight(false);
        else
            setLight(true);
    }

    @Override
    public void flashLight() {
        LimelightHelpers.setLEDMode_ForceBlink(name);
    }

    @Override
    public void setSnapshotMode(int snapVal) {
        // if (snapVal == 0)
        // LimelightHelpers.resetSnapshotMode(name);
        // else if (snapVal == 1)
        // LimelightHelpers.takeSnapshot(name);
        // else
        // assert false : " setSnapshotMode Integer out of bounds, accepted values =
        // 1,2";
    }

    public void setPiPMode(int pipVal) {
        switch (pipVal) {
            case 1:
                LimelightHelpers.setStreamMode_Standard(name);
                break;
            case 2:
                LimelightHelpers.setStreamMode_PiPMain(name);
                break;
            case 3:
                LimelightHelpers.setStreamMode_PiPSecondary(name);
                break;
            default:
                assert false : " setPiPMode Integer out of bounds, accepted values = 1,2,3";
                break;
        }
    }
}
