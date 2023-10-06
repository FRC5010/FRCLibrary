// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

/** Add your docs here. */
public class VisionValuesLimeLight extends VisionValues {
    private double centerX = 0.0;
    private double centerY = 0.0;
    private double horizontal = 0.0;
    private double vertical = 0.0;

    public VisionValuesLimeLight() {
        super();
    }

    public void averageValues(VisionValuesLimeLight rawValues, int maxCount) {
        super.averageValues(rawValues, maxCount);
        horizontal = rawValues.getHorizontal();
        vertical = rawValues.getVertical();
        centerX = rawValues.getCenterX();
        centerY = rawValues.getCenterY();
    }
    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
    }

    public double getHorizontal() {
        return horizontal;
    }

    public double getVertical() {
        return vertical;
    }

    public VisionValuesLimeLight setCenterY(double centerY) {
        this.centerY = centerY;
        return this;
    }

    public VisionValuesLimeLight setCenterX(double centerX) {
        this.centerX = centerX;
        return this;
    }

    public VisionValuesLimeLight setHorizontal(double horizontal) {
        this.horizontal = horizontal;
        return this;
    }

    public VisionValuesLimeLight setVertical(double vertical) {
        this.vertical = vertical;
        return this;
    }
}
