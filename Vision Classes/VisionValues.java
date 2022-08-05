/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

/**
 * Add your docs here.
 */
public class VisionValues {

    // the essential variables, stuff they already give me
    // TODO: If you ever get rid of a networkentry variable in opensight, update
    // vision classes

    private boolean valid = false;
    private double centerX = 0.0;
    private double centerY = 0.0;

    private double angleX = 0.0;
    private double angleY = 0.0;
    // distance
    private double distance = 0.0;
    
    // boundaries
    private double horizontal = 0.0;
    private double vertical = 0.0;

    private int count = 0;

    public VisionValues() {        
    }

    public VisionValues(boolean valid, double centerX, double centerY, double angleX, double angleY, double distance) {
        this.valid = valid;
        this.centerX = centerX;
        this.centerY = centerY;
        this.angleX = angleX;
        this.angleY = angleY;
        this.distance = distance;
    }

    public VisionValues(boolean valid, double centerX, double centerY, double angleX, double angleY, 
        double distance, double horizontal, double vertical) {
        this.valid = valid;
        this.centerX = centerX;
        this.centerY = centerY;
        this.angleX = angleX;
        this.angleY = angleY;
        this.distance = distance;
        this.horizontal = horizontal;
        this.vertical = vertical;
    }

    public boolean getValid() {
        return valid;
    }

    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
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

    public double getHorizontal() {
        return horizontal;
    }

    public double getVertical() {
        return vertical;
    }

    public void averageValues(VisionValues rawValues, int maxCount) {
        count++;
        count = Math.min(count, maxCount);
        valid = count >= maxCount;
        angleX = ((count - 1) * angleX + rawValues.getAngleX()) / count;
        angleY = ((count - 1) * angleY + rawValues.getAngleY()) / count;
        distance = ((count - 1) * distance + rawValues.getDistance()) / count;
        horizontal = rawValues.getHorizontal();
        vertical = rawValues.getVertical();
    }
}
