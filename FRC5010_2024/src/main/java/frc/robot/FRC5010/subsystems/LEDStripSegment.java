// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class LEDStripSegment {
    private int start, end;
    private Color color;
    private boolean active = false;
    public Function<Integer, Color8Bit> setLED = null;
    public Consumer<AddressableLEDBuffer> setLEDStrip = null;

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

    public void setLedAction(Consumer<AddressableLEDBuffer> ledFunction) {
        setLEDStrip = ledFunction;
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

    int m_rainbowFirstPixelHue = 180;

    public Consumer<AddressableLEDBuffer> rainbow() {
        m_rainbowFirstPixelHue = 180;
        return (led) -> {
            for (var i = start; i <= end; i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (m_rainbowFirstPixelHue + (i * 180 / (end - start))) % 180;
                // Set the value
                led.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
        };
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

    int ledPos = start;
    public Consumer<AddressableLEDBuffer> orbit(int r1, int g1, int b1, int r2, int g2, int b2, double percentLed) {
        int red1 = r1;
        int green1 = g1;
        int blue1 = b1;
        int red2 = r2;
        int green2 = g2;
        int blue2 = b2;
        int length = end - start;
        int numberOrbit = (int) ((double)length * percentLed);
        return (led) -> {
            for (int i = start; i <= end; i++) {
                if (i >= ledPos && (i - ledPos) < numberOrbit) {
                    led.setRGB(i % length, red2, green2, blue2);
                    if ((ledPos + numberOrbit) - length >= 0) {
                        for (int j = 0; j <= (ledPos + numberOrbit) - length; j++) {
                            led.setRGB(j, red2, green2, blue2);
                        }
                    }
                } else {
                    led.setRGB(i % length, red1, green1, blue1);
                }
            }

            // pushes the positon of the orbititing leds up by 1
            ledPos++;
            if (ledPos > end) {
                ledPos = start;
            }
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
