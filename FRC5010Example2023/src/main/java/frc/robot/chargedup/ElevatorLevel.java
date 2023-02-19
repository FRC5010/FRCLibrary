package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(Units.inchesToMeters(6),-20), 
  low(Units.inchesToMeters(0),-30), 
  medium(Units.inchesToMeters(30), 20), 
  high(Units.inchesToMeters(60),40);

  private double extension, pivot;

  private ElevatorLevel(double extension, double pivot) {
    this.extension = extension;
    this.pivot = pivot;
  }

  public double getExtenstionPosition() {
      return extension;
  }
  public double getPivotPosition() {
    return pivot;
  }
}