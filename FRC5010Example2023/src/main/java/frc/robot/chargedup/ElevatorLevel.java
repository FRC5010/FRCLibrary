package frc.robot.chargedup;

public enum ElevatorLevel {
  ground(1.2, -13),
  low(1.2, -8),
  medium(1.43, 27.7),
  high(1.743, 30);

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