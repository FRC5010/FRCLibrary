package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ArmLevel {
  ground(0),
  low(45),
  middle(45),
  home(105),
  high(140);

  private double pivot;

  private ArmLevel(double pivot) {
    this.pivot = pivot;
  }

  public double getPivotPosition() {
    return pivot;
  }
}