// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class LEDStripSegment {

    private int start, end;
    private String currentLedAction = "ON";
    private Color color;

    public LEDStripSegment(int start, int end, Color color) {
        this.start = start;
        this.end = end;
        this.color = color;
    }

    // solid color on
    public Color8Bit on() {
        return color.getColor8Bit();
    }

    public Color8Bit off() {
        return Color.OFF.getColor8Bit();
    }

    // blink based on time
    public Color8Bit blink(long onTime, long offTime) {
        if (System.currentTimeMillis() % (onTime + offTime) <= onTime) {
            return color.getColor8Bit();
        }
        return Color.OFF.getColor8Bit();
    }

    public Color8Bit flame(int scalar, int currLedPos) {

        if (currLedPos < scalar) {
            return color.getColor8BitAlpha(70 + (Math.random() * 40));
        } else if (currLedPos < scalar * 1.3) {
            return color.getColor8BitAlpha(30 + (Math.random() * 40));
        } else if (currLedPos < scalar * 2) {
            return color.getColor8BitAlpha(0 + (Math.random() * 40));
        }
        return color.getColor8BitAlpha(((Math.random() * 30) - 29) * 20);
    }

    public void chase() {

    }

    public String getCurrentAction() {
        return currentLedAction;
    }
}
