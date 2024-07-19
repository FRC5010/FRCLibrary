/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frc5010.common.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  private long currTime;
  private long startTime;
  private long delayMs;
  /** Creates a new LedController. */
  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDBuffer m_ledOff;
  private int m_rainbowFirstPixelHue = 180;

  private boolean ledOn;
  private String status = "off";

  private int red1;
  private int green1;
  private int blue1;
  private int red2;
  private int green2;
  private int blue2;
  private int numberOrbit;
  private int ledPos = 0;

  private boolean ledConeMode = false;

  public LedSubsystem(int port, int length) {
    // init method, sets up the led strip and if you want it to be one solid color
    // you would do that here
    // you can still change it later
    m_led = new AddressableLED(port);

    m_ledBuffer = new AddressableLEDBuffer(length); // standard 300
    m_ledOff = new AddressableLEDBuffer(length);
    m_led.setLength(m_ledBuffer.getLength());

    // setting all of the leds to orange by using a for loop
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledOff.setRGB(i, 0, 0, 0);
    }
    // taking the data created above and inserting it into the leds
    m_led.setData(m_ledBuffer);
    m_led.start();
    ledPos = 0;
  }

  @Override
  public void periodic() {
    currTime = System.currentTimeMillis();
    // Runs the blueSnake method which changes the m_ledBuffer, then the m_led is
    // set to the data that was just created

    // this method is esscentially only for controlling blink
    // System.out.println(status);
    switch (status) {
      case "rainbow":
        rainbow();
        break;
      case "orbit":
        orbit();
        break;
      case "blink":
        blink();
        break;
      default:
        break;
    }
  }

  private void rainbow() { // completely not needed but proof of concept if we want?
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_led.setData(m_ledBuffer);
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void blink() {
    if (currTime - startTime >= delayMs) {
      if (ledOn) {
        m_led.setData(m_ledOff);
        ledOn = false;
      } else {
        m_led.setData(m_ledBuffer);
        ledOn = true;
      }
      startTime = currTime;
    }
  }

  private void orbit() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i >= ledPos && (i - ledPos) < numberOrbit) {
        m_ledBuffer.setRGB(i % m_ledBuffer.getLength(), red2, green2, blue2);
        if ((ledPos + numberOrbit) - m_ledBuffer.getLength() >= 0) {
          for (int j = 0; j <= (ledPos + numberOrbit) - m_ledBuffer.getLength(); j++) {
            m_ledBuffer.setRGB(j, red2, green2, blue2);
          }
        }
      } else {
        m_ledBuffer.setRGB(i % m_ledBuffer.getLength(), red1, green1, blue1);
      }
    }
    m_led.setData(m_ledBuffer);

    // pushes the positon of the orbititing leds up by 1
    ledPos++;
    if (ledPos > m_ledBuffer.getLength()) {
      ledPos = 0;
    }
  }

  public void setRainbow() {
    status = "rainbow";
  }

  public void setOrbit(int r1, int g1, int b1, int r2, int g2, int b2, double percentLed) {
    red1 = r1;
    green1 = g1;
    blue1 = b1;
    red2 = r2;
    green2 = g2;
    blue2 = b2;
    numberOrbit = (int) ((double) m_ledBuffer.getLength() * percentLed);
    status = "orbit";
  }

  public void speed(double power) {
    // this method shows that you can use the speed of something to change the leds,
    // and its pretty simple to do
    int amountOn = Math.abs((int) ((double) m_ledBuffer.getLength() * power));
    int leds = m_ledBuffer.getLength() - 1;
    for (int i = 0; i < amountOn; i++) {
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    for (int i = leds; i >= amountOn; i--) {
      m_ledBuffer.setRGB(i, 255, 20, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setSolidColor(int red, int green, int blue) {
    status = "solid";
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setSolidColorPercent(int red, int green, int blue, double percent) {
    status = "solid_percent";
    int percentLength = (int) ((double) m_ledBuffer.getLength() * percent);
    for (int i = 0; i < percentLength; i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }

    for (int i = percentLength; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void setBlink(int red, int green, int blue, long delayMs) {
    this.delayMs = delayMs;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }
    ledOn = true;
    m_led.setData(m_ledBuffer);
    startTime = currTime;
    status = "blink";
  }

  public void off() {
    setSolidColor(0, 0, 0);
  }

  public void togglePickUp() {
    ledConeMode = !ledConeMode;
  }

  public boolean getLedConeMode() {
    return ledConeMode;
  }
}
