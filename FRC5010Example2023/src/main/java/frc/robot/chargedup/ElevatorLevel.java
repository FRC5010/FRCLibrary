package frc.robot.chargedup;

public enum ElevatorLevel {
  ground, low, medium, high;
  public double getExtenstionPosition() {
    switch (this) {
      case ground:
        return 0;
      case low:
        return 0;
      case medium:
        return 0;
      case high:
        return 0;
      default:
        return 0;

    }

  }
  public double getPivotPosition() {
    switch (this) {
      case ground:
        return 0;
      case low:
        return 0;
      case medium:
        return 0;
      case high:
        return 0;
      default:
        return 0;

    }
  }
}