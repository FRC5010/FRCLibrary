package frc.robot.chargedup;

public enum ElevatorLevel {
  ground(1.184, PivotSubsystem.pivotOffset + 0.55),
  low(1.2, -8),
  loading(0.7325, 23.5052),
  medium(1.3985, 26.14),
  high(1.6525, 31.805);

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