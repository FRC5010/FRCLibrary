// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewLedSubsystem extends SubsystemBase {
  /** Creates a new NewLedSubsystem. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDBuffer m_ledOff;

  private ArrayList<LEDStripSegment> ledStripSegments; // Might need to be implemented differently

  public NewLedSubsystem(int port, int length) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_ledOff = new AddressableLEDBuffer(length);

    m_led.setLength(m_ledBuffer.getLength());

    this.ledStripSegments.add(new LEDStripSegment(0, length, Color.OFF));

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledOff.setRGB(i, 0, 0, 0);
    }
    // taking the data created above and inserting it into the leds
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
