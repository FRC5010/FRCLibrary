// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.FRC5010.Vision.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class VisionLimeLightLib extends VisionSystem {

    public VisionLimeLightLib(String name, int colIndex, AprilTagFieldLayout fieldLayout, Transform3d cameraToRobot) {
        super("limelight-" + name, colIndex, fieldLayout);
        this.cameraToRobot = cameraToRobot;
        init();
    }

    public VisionLimeLightLib(String name, DoubleSupplier camHeight, DoubleSupplier camAngle, DoubleSupplier cameraDistance, double targetHeight, int colIndex,
           AprilTagFieldLayout fieldLayout, String driverTabeName, Transform3d cameraToRobot) {
        super("limelight-" + name, camHeight, camAngle, cameraDistance, targetHeight, colIndex, fieldLayout, driverTabeName);
        this.cameraToRobot = cameraToRobot;
    }

    protected void init() {
    }

    public void update() {
        updateViaNetworkTable(name);
    }

    private void updateViaNetworkTable(String name) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        rawValues.clearValues();
        if (results.targetingResults.valid) {

            Pose2d robotPose2d = RobotContainer.getAlliance().equals(Alliance.Red)
                    ? LimelightHelpers.getBotPose2d_wpiRed(name)
                    : LimelightHelpers.getBotPose2d_wpiBlue(name);
            updateValues(rawValues,
                    () -> LimelightHelpers.getTX(name),
                    () -> LimelightHelpers.getTY(name),
                    () -> LimelightHelpers.getTA(name),
                    () -> LimelightHelpers.getTV(name),
                    () -> Timer.getFPGATimestamp() -
                            ((LimelightHelpers.getLatency_Pipeline(name) +
                                    LimelightHelpers.getLatency_Capture(name)) * 0.001),
                    () -> Double.valueOf(LimelightHelpers.getFiducialID(name)).intValue(),
                    () -> {
                        if (results.targetingResults.targets_Fiducials.length == 0) {
                            return null;
                        }
                        return fieldLayout
                                .getTagPose(
                                        ((int) results.targetingResults.targets_Fiducials[0].fiducialID))
                                .orElse(null);
                    },
                    () -> robotPose2d);
        }
    }

    @Override
    public Vector<N3> getStdVector(double distance) {
        return VecBuilder.fill(.7,.7,9999999);
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
