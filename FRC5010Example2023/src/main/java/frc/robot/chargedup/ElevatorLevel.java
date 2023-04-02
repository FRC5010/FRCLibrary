package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(1.373, -11.53),
  low(1.2, -8),
  loading(0.9797, 26.2986),
  medium(1.4967, 26.7928),
  high(2.0435, 30.657),
  auto(Units.inchesToMeters(30), 31.583);

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