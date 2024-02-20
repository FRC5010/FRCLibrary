// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SegmentedLedSystem extends SubsystemBase {
  /** Creates a new NewLedSubsystem. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDBuffer m_ledOff;

  private Color currColor = Color.OFF;

  private Map<String, LEDStripSegment> ledStripSegments; // Might need to be implemented differently

  public SegmentedLedSystem(int port, int length) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_ledOff = new AddressableLEDBuffer(length);

    m_led.setLength(m_ledBuffer.getLength());

    LEDStripSegment all = new LEDStripSegment(0, length, currColor);
    this.ledStripSegments.put("All", all);
    all.setActive(false);
    all.setLedAction(all.on());
    all.setColor(Color.ORANGE);

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledOff.setRGB(i, 0, 0, 0);
    }
    // taking the data created above and inserting it into the leds
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    for (String name : ledStripSegments.keySet()) {
      LEDStripSegment segment = ledStripSegments.get(name);
      if (segment.isActive()) {
        for (int i = segment.start(); i <= segment.end(); ++i) {
          m_ledBuffer.setLED(i, segment.setLED.apply(i));
        }
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void addLedSegment(String name, int start, int end, Color color) {
    ledStripSegments.put(name, new LEDStripSegment(0, 0, color));
  }

  public void setLedSegment(String name, Function<Integer, Color8Bit> state) {
    ledStripSegments.get(name).setLedAction(state);
  }

  public void setWholeStripState(Function<Integer, Color8Bit> state) {
    ledStripSegments.values().stream().forEach(it -> it.setActive(false));
    ledStripSegments.get("All").setLedAction(state);
  }
}
