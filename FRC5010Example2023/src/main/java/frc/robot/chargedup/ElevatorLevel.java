package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(1.469,-8), 
  low(ElevatorSubsystem.kMinElevatorHeight + Units.inchesToMeters(10), 0), 
  medium(1.412, 23), 
  high(1.81,27.94);

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