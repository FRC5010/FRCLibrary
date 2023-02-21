package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;

public enum ElevatorLevel {
  ground(ElevatorSubsystem.kMinElevatorHeight,0), 
  low(ElevatorSubsystem.kMinElevatorHeight,10), 
  medium(ElevatorSubsystem.kMinElevatorHeight + Units.inchesToMeters(20), 20), 
  high(ElevatorSubsystem.kMaxElevatorHeight + Units.inchesToMeters(40),30);

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