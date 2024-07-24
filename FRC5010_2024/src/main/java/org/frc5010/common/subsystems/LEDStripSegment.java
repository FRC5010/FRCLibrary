// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.Consumer;
import java.util.function.Function;

/** Add your docs here. */
public class LEDStripSegment {
  private int start, end;
  private Color8Bit color;
  private boolean active = false;
  private boolean needsUpdate = false;
  public Function<Integer, Color8Bit> setLED = null;
  public Consumer<AddressableLEDBuffer> setLEDStrip = null;

  public LEDStripSegment(int start, int end, Color color) {
    this.start = start;
    this.end = end;
    this.color = color.getColor8Bit();
  }

  public LEDStripSegment(int start, int end, Color8Bit color) {
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
    setLEDStrip = null;
  }

  public void setLedAction(Consumer<AddressableLEDBuffer> ledFunction) {
    setLEDStrip = ledFunction;
    setLED = null;
  }

  public LEDStripSegment setActive(boolean active) {
    this.active = active;
    this.needsUpdate = true;
    return this;
  }

  public boolean isActive() {
    return active;
  }

  public void setNeedsUpdate(boolean update) {
    needsUpdate = update;
  }

  public boolean needsUpdate() {
    return needsUpdate;
  }

  public LEDStripSegment setColor(Color color) {
    this.color = color.getColor8Bit();
    needsUpdate = true;
    return this;
  }

  public Function<Integer, Color8Bit> getCurrentAction() {
    return setLED;
  }

  // solid color on
  public void on() {
    setLedAction((Integer i) -> color);
    needsUpdate = true;
  }

  int m_rainbowFirstPixelHue = 180;

  public void rainbow() {
    m_rainbowFirstPixelHue = 180;
    needsUpdate = true;
    setLedAction(
        (led) -> {
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
          needsUpdate = true;
        });
  }

  public void off() {
    setLedAction((Integer i) -> Color.OFF.getColor8Bit());
    needsUpdate = true;
  }

  // blink based on time
  public void blink(long onTime, long offTime) {
    needsUpdate = true;
    setLedAction(
        (Integer i) -> {
          needsUpdate = true;
          if (System.currentTimeMillis() % (onTime + offTime) <= onTime) {
            return color;
          }
          return Color.OFF.getColor8Bit();
        });
  }

  public void flame(int scalar) {
    needsUpdate = true;
    setLedAction(
        (Integer currLedPos) -> {
          needsUpdate = true;
          if (currLedPos < scalar) {
            return Color.getColor8BitAlpha(color, 50 + (Math.random() * 40));
          } else if (currLedPos < scalar * 1.3) {
            return Color.getColor8BitAlpha(color, 30 + (Math.random() * 40));
          } else if (currLedPos < scalar * 2) {
            return Color.getColor8BitAlpha(color, Math.random() * 40);
          }
          return Color.getColor8BitAlpha(color, ((Math.random() * 30) - 29) * 20);
        });
  }

  int ledPos = start;

  public void orbit(int r1, int g1, int b1, int r2, int g2, int b2, double percentLed) {
    int red1 = r1;
    int green1 = g1;
    int blue1 = b1;
    int red2 = r2;
    int green2 = g2;
    int blue2 = b2;
    int length = end - start;
    int numberOrbit = (int) ((double) length * percentLed);
    needsUpdate = true;
    setLedAction(
        (AddressableLEDBuffer led) -> {
          needsUpdate = true;
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
        });
  }

  public void chase(boolean rainbow) {
    needsUpdate = true;
    int length = end - start;
    setLedAction(
        (Integer i) -> {
          int pos = i - start;
          needsUpdate = true;
          long currentTime = System.currentTimeMillis();
          long chasePos = (length * (currentTime % 1000)) / 1000;
          long timeSecs = currentTime / 1000;
          boolean evenOdd = timeSecs == (2 * (timeSecs / 2));
          if (rainbow && i == end) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / (end - start))) % 180;
            // Set the value
            color = Color.fromHSV(hue, 255, 128);
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
          }
          if (evenOdd) chasePos = length - chasePos;
          if (chasePos == pos) {
            return color;
          } else {
            double colorPer = 0;
            if (pos < chasePos) {
              colorPer = 1.0 - ((double) (chasePos - pos) / (chasePos - start));
            } else {
              colorPer = 1.0 - ((double) (pos - chasePos) / (end - chasePos));
            }
            int red = (int) Math.round(color.red * colorPer);
            int green = (int) Math.round(color.green * colorPer);
            int blue = (int) Math.round(color.blue * colorPer);
            return new Color8Bit(red, green, blue);
          }
        });
  }
}
