package frc.robot.chargedup;

public enum ArmLevel {
  ground(4),
  low(45),
  middle(45),
  home(105),
  high(140);

  private double pivot;

  private ArmLevel(double pivot) {
    this.pivot = pivot;
  }

  public double getPivotPosition() {
    return pivot;
  }
}