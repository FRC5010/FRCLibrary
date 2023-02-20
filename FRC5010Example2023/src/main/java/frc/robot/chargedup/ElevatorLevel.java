package frc.robot.chargedup;

public enum ElevatorLevel {
  ground(0,0), 
  low(0,0), 
  medium(0,0), 
  high(0,0);

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