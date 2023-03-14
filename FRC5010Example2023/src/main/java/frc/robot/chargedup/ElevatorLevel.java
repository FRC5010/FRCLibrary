package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(1.2,-10), 
  low(1.2, -8), 
  medium(1.43, 28.3), 
  high(1.743, 33);

  private double extension, pivot;

  private ElevatorLevel(double extension, double pivot) {
    this.extension = extension;
    this.pivot = pivot;
  }

  public double getExtensionPosition() {
      return extension;
  }
  public double getPivotPosition() {
    return pivot;
  }
}