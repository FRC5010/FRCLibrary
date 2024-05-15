// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class ButtonBoard extends Controller {

  // private Joystick joystick;
  // private boolean singleControllerMode;
  private List<JoystickButton> BUTTONS = new ArrayList<>();
  // private POVButton UP, RIGHT, DOWN, LEFT;
  // private Axis AxisX, AxisY;

  public ButtonBoard(int port) {
    super(port);
  }

  public void setYAxis(Axis yAxis) {
    super.setLeftYAxis(yAxis);
  }

  public void setXAxis(Axis xAxis) {
    super.setLeftXAxis(xAxis);
  }

  public Axis createYAxis() {
    return super.createLeftYAxis();
  }

  public Axis createXAxis() {
    return super.createLeftXAxis();
  }

  public double getYAxis() {
    return super.getLeftYAxis();
  }

  public double getXAxis() {
    return super.getLeftXAxis();
  }

  public void createButtons(int numberOfButtons) {
    for (int i = 1; i < numberOfButtons; i++) {
      BUTTONS.add(super.createCustomButton(i));
    }
  }

  public JoystickButton getButton(int button) {
    return BUTTONS.get(button - 1);
  }
}
