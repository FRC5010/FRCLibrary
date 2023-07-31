// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

/** Add your docs here. */
public class VisionValuesPhotonCamera extends VisionValues {
    private int fiducialId = 0;

    public int getFiducialId() {
        return fiducialId;
    }

    public VisionValuesPhotonCamera setFiducialId(int id) {
        fiducialId = id;
        return this;
    }
}
