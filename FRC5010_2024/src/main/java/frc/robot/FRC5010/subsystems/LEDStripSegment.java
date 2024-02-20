// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import java.util.function.Function;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class LEDStripSegment {
    private int start, end;
    private Color color;
    private boolean active = false;
    public Function<Integer, Color8Bit> setLED;

    public LEDStripSegment(int start, int end, Color color) {
        this.start = start;
        this.end = end;
        this.color = color;
    }

    public int start() {
        return start;
    }

    public int end() {
        return end;
    }

    public void setLedAction(Function<Integer, Color8Bit> ledFunction) {
        setLED = ledFunction;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return active;
    }
    
    // solid color on
    public Function<Integer, Color8Bit> on() {
        return (i) -> color.getColor8Bit();
    }

    public Function<Integer, Color8Bit> off() {
        return (i) -> Color.OFF.getColor8Bit();
    }

    // blink based on time
    public Function<Integer, Color8Bit> blink(long onTime, long offTime) {
        return (i) -> {
            if (System.currentTimeMillis() % (onTime + offTime) <= onTime) {
                return color.getColor8Bit();
            }
            return Color.OFF.getColor8Bit();
        };
    }

    public Function<Integer, Color8Bit> flame(int scalar, int currLedPos) {
        return (i) -> {
            if (currLedPos < scalar) {
                return color.getColor8BitAlpha(50 + (Math.random() * 40));
            } else if (currLedPos < scalar * 1.3) {
                return color.getColor8BitAlpha(30 + (Math.random() * 40));
            } else if (currLedPos < scalar * 2) {
                return color.getColor8BitAlpha(Math.random() * 40);
            }
            return color.getColor8BitAlpha(((Math.random() * 30) - 29) * 20);
        };
    }

    public Function<Integer, Color8Bit> chase(int curLedPos) {
        // TODO
        int chasePosition = 0;
        int chaseDirection = 1;
        return (i) -> {
            return new Color8Bit();
        };
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public Function<Integer, Color8Bit> getCurrentAction() {
        return setLED;
    }
}
