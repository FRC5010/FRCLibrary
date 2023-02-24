package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(ElevatorSubsystem.kMinElevatorHeight,-10), 
  low(ElevatorSubsystem.kMinElevatorHeight,0), 
  medium(ElevatorSubsystem.kMinElevatorHeight + Units.inchesToMeters(20), 10), 
  high(ElevatorSubsystem.kMaxElevatorHeight + Units.inchesToMeters(40),20);

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