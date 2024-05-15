// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import org.frc5010.common.arch.GenericSubsystem;

public class SegmentedLedSystem extends GenericSubsystem {
  /** Creates a new NewLedSubsystem. */
  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDBuffer m_ledOff;
  private boolean needsUpdate = false;

  private Color currColor = Color.OFF;
  public final String ALL = "All";

  private Map<String, LEDStripSegment> ledStripSegments =
      new HashMap<>(); // Might need to be implemented differently
  private List<MechanismLigament2d> simLEDs = new ArrayList<>();

  public SegmentedLedSystem(int port, int length, Mechanism2d simulator) {
    setMechSimulation(simulator);
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_ledOff = new AddressableLEDBuffer(length);

    m_led.setLength(m_ledBuffer.getLength());

    LEDStripSegment all = new LEDStripSegment(0, length - 1, currColor);
    this.ledStripSegments.put(ALL, all);
    all.setActive(false);
    all.on();
    all.setColor(currColor);

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledOff.setRGB(i, 0, 0, 0);
      MechanismRoot2d ledRoot = mechanismSimulation.getRoot("LEDRoot " + i, i / 100.0, 1.0);
      MechanismLigament2d led =
          new MechanismLigament2d("LED " + 1, 0.02, -90, 2, Color.OFF.getColor8Bit());
      ledRoot.append(led);
      simLEDs.add(led);
    }
    // taking the data created above and inserting it into the leds
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    for (String name : ledStripSegments.keySet()) {
      LEDStripSegment segment = ledStripSegments.get(name);
      if (segment.isActive() && segment.needsUpdate()) {
        needsUpdate |= segment.needsUpdate();
        segment.setNeedsUpdate(false);
        if (null != segment.setLEDStrip) {
          segment.setLEDStrip.accept(m_ledBuffer);
          if (RobotBase.isSimulation()) {
            for (int i = segment.start(); i <= segment.end(); ++i) {
              simLEDs.get(i).setColor(m_ledBuffer.getLED8Bit(i));
            }
          }
        } else {
          for (int i = segment.start(); i <= segment.end(); ++i) {
            Color8Bit color = segment.setLED.apply(i - segment.start());
            m_ledBuffer.setLED(i, color);
            if (RobotBase.isSimulation()) {
              simLEDs.get(i).setColor(color);
            }
          }
        }
      }
    }
    if (needsUpdate) {
      m_led.setData(m_ledBuffer);
      needsUpdate = false;
    }
  }

  public LEDStripSegment getStrip(String name) {
    return ledStripSegments.get(name);
  }

  public void addLedSegment(String name, int start, int end, Color color) {
    ledStripSegments.put(name, new LEDStripSegment(0, 0, color));
  }

  public void setLedSegmentAction(String name, Function<Integer, Color8Bit> state) {
    ledStripSegments.get(name).setLedAction(state);
  }

  public void setLedSegmentColor(String name, Color color) {
    ledStripSegments.get(name).setColor(color);
  }

  public void setLedSegmentActive(String name, boolean active) {
    ledStripSegments.get(name).setActive(active);
  }

  public void setWholeStripState(Function<Integer, Color8Bit> state) {
    ledStripSegments.values().stream().forEach(it -> it.setActive(false));
    ledStripSegments.get(ALL).setLedAction(state);
    ledStripSegments.get(ALL).setActive(true);
    needsUpdate = true;
  }

  public void setWholeStripState(Consumer<AddressableLEDBuffer> state) {
    ledStripSegments.values().stream().forEach(it -> it.setActive(false));
    ledStripSegments.get(ALL).setLedAction(state);
    ledStripSegments.get(ALL).setActive(true);
    needsUpdate = true;
  }
}
