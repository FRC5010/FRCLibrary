// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class LEDStripSegment {

    private int start, end;
    private String currentLedAction = "OFF";
    private Color color;

    public LEDStripSegment(int start, int end, Color color) {
        this.start = start;
        this.end = end;
        this.color = color;
    }

    // solid color on
    public Color8Bit on() {
        currentLedAction = "ON";
        return color.getColor8Bit();
    }

    public Color8Bit off() {
        currentLedAction = "OFF";
        return Color.OFF.getColor8Bit();
    }

    // blink based on time
    public Color8Bit blink(long onTime, long offTime) {
        currentLedAction = "BLINK";
        if (System.currentTimeMillis() % (onTime + offTime) <= onTime) {
            return color.getColor8Bit();
        }
        return Color.OFF.getColor8Bit();
    }

    public Color8Bit flame(int scalar, int currLedPos) {
        currentLedAction = "FLAME";
        if (currLedPos < scalar) {
            return color.getColor8BitAlpha(50 + (Math.random() * 40));
        } else if (currLedPos < scalar * 1.3) {
            return color.getColor8BitAlpha(30 + (Math.random() * 40));
        } else if (currLedPos < scalar * 2) {
            return color.getColor8BitAlpha(Math.random() * 40);
        }
        return color.getColor8BitAlpha(((Math.random() * 30) - 29) * 20);
    }

    public void chase() {
        currentLedAction = "CHASE";
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public String getCurrentAction() {
        return currentLedAction;
    }
}
