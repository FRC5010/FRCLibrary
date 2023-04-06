package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(1.373, -10.1),
  low(1.2, -8),
  loading(0.9797, 26.2986),
  medium(1.4967, 26.7928),
  high(1.970, 30.657),
  auto(Units.inchesToMeters(30), 31.583),
  conePickUp(1.47, ground.getPivotPosition());

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