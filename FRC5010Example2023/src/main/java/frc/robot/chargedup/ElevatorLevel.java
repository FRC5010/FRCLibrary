package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(1.1,-13.5), 
  low(1.1, -8), 
  medium(1.43, 28.3), 
  high(1.7, 33);

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